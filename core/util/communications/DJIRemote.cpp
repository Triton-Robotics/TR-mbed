#include "DJIRemote.h"

using namespace std::chrono;

Remote::Remote(PinName dbus) : receiver(NC, dbus) {

    //printf("YEETUSdeletus\n");
    receiver.set_baud(100000);
    receiver.set_format(8, BufferedSerial::Even, 1);
    receiver.set_blocking(false);
    //receiver.set_flow_control(BufferedSerial::Disabled);
    readTimer.start();
    //printf("Remote created \n");
}

void Remote::initialize(){
}

void Remote::read(){
    
    // Read next byte if available and more needed for the current packet
    // Check disconnect timeout
    if ((duration_cast<milliseconds>(readTimer.elapsed_time()).count() - lastRead) > REMOTE_DISCONNECT_TIMEOUT)
    {
        // printf("Remote disconnected \n");
        connected = false;  // Remote no longer connected
        reset();            // Reset current remote values
    }
    uint8_t data;  // Next byte to be read
    // Read next byte if available and more needed for the current packet
    while (receiver.read(&data, 1) && (currentBufferIndex < REMOTE_BUF_LEN))
    {   
        //printf("%x \t", data);
        rxBuffer[currentBufferIndex] = data;
        currentBufferIndex++;
        lastRead = duration_cast<milliseconds>(readTimer.elapsed_time()).count();
    }
    //printf("\n");
    // Check read timeout
    if ((duration_cast<milliseconds>(readTimer.elapsed_time()).count() - lastRead) > REMOTE_READ_TIMEOUT)
    {
        clearRxBuffer();
    }
    // Parse buffer if all 18 bytes are read
    if (currentBufferIndex >= REMOTE_BUF_LEN)
    {
        connected = true;
        parseBuffer();
        clearRxBuffer();
    }

}

__attribute__((unused)) bool Remote::isConnected() const { return connected; }

float apply_deadzone(float num){
    const float deadzone = 0.05;
    return fabs(num) < deadzone ? 0.0 : num; 
}

float Remote::getChannel(Channel ch) const{
    switch (ch)
    {
        case Channel::RIGHT_HORIZONTAL:
            return  apply_deadzone(float(remote.rightHorizontal) / STICK_MAX_VALUE);
        case Channel::RIGHT_VERTICAL:
            return apply_deadzone(float(remote.rightVertical) / STICK_MAX_VALUE);
        case Channel::LEFT_HORIZONTAL:
            return apply_deadzone(float(remote.leftHorizontal) / STICK_MAX_VALUE);
        case Channel::LEFT_VERTICAL:
            return apply_deadzone(float(remote.leftVertical) / STICK_MAX_VALUE);
    }
    return 0;
}


int16_t Remote::getChannelInt(Channel ch) const{
    switch (ch){
        case Channel::RIGHT_HORIZONTAL:
            return remote.rightHorizontal;

        case Channel::RIGHT_VERTICAL:
            return remote.rightVertical;

        case Channel::LEFT_HORIZONTAL:
            return remote.leftHorizontal;

        case Channel::LEFT_VERTICAL:
            return remote.leftVertical;
    }
    return 0;
}


float Remote::getChassisX()
{
    return ((remote.leftHorizontal / 660.0) + 
            (keyPressed(Key::SHIFT) ? 0.5 : 1 * 
                ((keyPressed(Key::W) ? 1 : 0) + 
                 (keyPressed(Key::S) ? -1 : 0))
            ));
}

float Remote::getChassisY()
{
    return ((remote.leftVertical / 660.0) + 
            ((keyPressed(Key::SHIFT) ? 0.5 : 1) * 
                ((keyPressed(Key::A) ? 1 : 0) + 
                (keyPressed(Key::D) ? -1 : 0)
                )
            ));
}


float Remote::getYaw()
{
    // TODO: make this a helper fn
    float normal_rh = remote.rightHorizontal / 660.0;
    normal_rh = abs(normal_rh) > 0.05 ? normal_rh : 0;
    return ((normal_rh) * JOYSTICK_SENSITIVITY_YAW_DPS + 
        remote.mouse.y * MOUSE_SENSITIVITY_YAW_DPS);
}

float Remote::getPitch()
{
    float normal_rv = remote.rightVertical / 660.0;
    normal_rv = abs(normal_rv) > 0.05 ? normal_rv : 0;
    return ((normal_rv) * JOYSTICK_SENSITIVITY_PITCH_DPS + 
            remote.mouse.y * MOUSE_SENSITIVITY_PITCH_DPS);
}

Remote::SwitchState Remote::getSwitch(Switch sw) const{
    switch (sw){
        case Switch::LEFT_SWITCH:
            return remote.leftSwitch;

        case Switch::RIGHT_SWITCH:
            return remote.rightSwitch;
    }

    return SwitchState::UNKNOWN;
}

void Remote::printAxisData() const{
    printf("%d %d %d %d\n", getChannelInt(Remote::Channel::LEFT_HORIZONTAL), getChannelInt(Remote::Channel::LEFT_VERTICAL), getChannelInt(Remote::Channel::RIGHT_HORIZONTAL), getChannelInt(Remote::Channel::RIGHT_VERTICAL));
}

void Remote::dumpInfo() const{
    printf("%d %d %d %d %d %d\n", 
        getChannelInt(Remote::Channel::LEFT_HORIZONTAL), 
        getChannelInt(Remote::Channel::LEFT_VERTICAL), 
        getChannelInt(Remote::Channel::RIGHT_HORIZONTAL), 
        getChannelInt(Remote::Channel::RIGHT_VERTICAL),
        static_cast<int>(getSwitch(Switch::LEFT_SWITCH)), 
        static_cast<int>(getSwitch(Switch::RIGHT_SWITCH))
    );
}

int16_t Remote::getMouseX() const { return remote.mouse.x; }

int16_t Remote::getMouseY() const { return remote.mouse.y; }

int16_t Remote::getMouseZ() const { return remote.mouse.z; }

bool Remote::getMouseL() const { return remote.mouse.l; }

bool Remote::getMouseR() const { return remote.mouse.r; }

bool Remote::keyPressed(Key key) const { return (remote.key & (1 << (uint8_t)key)) != 0; }

int16_t Remote::getWheel() const { return remote.wheel; }

void Remote::switchToState(RemoteInfo *remote){

    switch ((int)remote -> leftSwitch) {
        case 1:
            remote -> leftSwitch = SwitchState::UP;
            break;
        case 3:
            remote -> leftSwitch = SwitchState::MID;
            break;
        case 2:
            remote -> leftSwitch = SwitchState::DOWN;
            break;
        default:
            remote -> leftSwitch = SwitchState::UNKNOWN;
            break;
    }

    switch ((int)remote -> rightSwitch) {
        case 1:
            remote -> rightSwitch = SwitchState::UP;
            break;
        case 3:
            remote -> rightSwitch = SwitchState::MID;
            break;
        case 2:
            remote -> rightSwitch = SwitchState::DOWN;
            break;
        default:
            remote -> rightSwitch = SwitchState::UNKNOWN;
            break;
    }

}

bool Remote::badData(const uint8_t rxBuffer[], RemoteInfo *remote){

    auto lSwitch = SwitchState(((rxBuffer[5] >> 4) & 0x000C) >> 2);
    auto rSwitch = SwitchState(((rxBuffer[5] >> 4) & 0x0003));

    int16_t rh = ((int16_t) rxBuffer[0] | ((int16_t) rxBuffer[1] << 8)) & 0x07FF;
    int16_t rv = (((int16_t) rxBuffer[1] >> 3) | ((int16_t) rxBuffer[2] << 5)) & 0x07FF;
    int16_t lh = (((int16_t) rxBuffer[2] >> 6) | ((int16_t) rxBuffer[3] << 2) | ((int16_t) rxBuffer[4] << 10)) & 0x07FF;
    int16_t lv = (((int16_t) rxBuffer[4] >> 1) | ((int16_t) rxBuffer[5] << 7)) & 0x07FF;

    if(unfiltered)
        printf("%d %d %d %d\n", rh, rv, lh, lv);

    if(!(bool(SwitchState(lSwitch)) and bool(SwitchState(rSwitch))))
        return true;

    int16_t joysticks[4] = {rh, rv, lh, lv};

    for(int16_t axis: joysticks) {
        if (abs(axis - 1024) > 660)
            return true;
    }

    remote -> rightHorizontal = rh;
    remote -> rightVertical = rv;
    remote -> leftHorizontal = lh;
    remote -> leftVertical = lv;

    remote -> leftSwitch = lSwitch;
    remote -> rightSwitch = rSwitch;

    switchToState(remote);

    return false;
}

void Remote::parseBuffer(){
    // values implemented by shifting bits across based on the dr16
    // values documentation and code created last year

    if(!badData(rxBuffer, &remote)) {

        remote.rightHorizontal -= 1024;
        remote.rightVertical -= 1024;
        remote.leftHorizontal -= 1024;
        remote.leftVertical -= 1024;

        remote.mouse.x = ((int16_t)rxBuffer[6]) | ((int16_t)rxBuffer[7] << 8);
        remote.mouse.y = ((int16_t)rxBuffer[8]) | ((int16_t)rxBuffer[9] << 8);
        remote.mouse.z = ((int16_t)rxBuffer[10]) | ((int16_t)rxBuffer[11] << 8);

        remote.mouse.l = static_cast<bool>(rxBuffer[12]);
        remote.mouse.r = static_cast<bool>(rxBuffer[13]);

        remote.key = ((int16_t)rxBuffer[14]) | ((int16_t)rxBuffer[15] << 8);
        remote.wheel = (((int16_t)rxBuffer[16]) | ((int16_t)rxBuffer[17] << 8)) - 1024;


        // switches on the dji remote - their input is registered

        remote.updateCounter++;
    }
}

void Remote::clearRxBuffer()
{
    // Reset bytes read counter
    currentBufferIndex = 0;
    // Clear remote rxBuffer
    for (int i = 0; i < REMOTE_BUF_LEN; i++)
    {
        rxBuffer[i] = 0;
    }
    // Clear Usart1 rxBuffer
    receiver.sync();

    uint8_t data;
    while (receiver.readable())
    {
        receiver.read(&data, 1);
    }
}

void Remote::reset()
{
    remote.rightHorizontal = 0;
    remote.rightVertical = 0;
    remote.leftHorizontal = 0;
    remote.leftVertical = 0;
    remote.leftSwitch = SwitchState::UNKNOWN;
    remote.rightSwitch = SwitchState::UNKNOWN;
    remote.mouse.x = 0;
    remote.mouse.y = 0;
    remote.mouse.z = 0;
    remote.mouse.l = false;
    remote.mouse.r = false;
    remote.key = 0;
    remote.wheel = 0;
    clearRxBuffer();

}

uint32_t Remote::getUpdateCounter() const { return remote.updateCounter; }

