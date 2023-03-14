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

void Remote::initialize()
{
}

void Remote::read()
{
    
    // Read next byte if available and more needed for the current packet
    // Check disconnect timeout
    if ((duration_cast<milliseconds>(readTimer.elapsed_time()).count() - lastRead) > REMOTE_DISCONNECT_TIMEOUT)
    {
        printf("Remote disconnected \n");
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
    printf("\n");
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

float Remote::getChannel(Channel ch) const
{
    switch (ch)
    {
        case Channel::RIGHT_HORIZONTAL:
            return float(remote.rightHorizontal) / STICK_MAX_VALUE;
        case Channel::RIGHT_VERTICAL:
            return float(remote.rightVertical) / STICK_MAX_VALUE;
        case Channel::LEFT_HORIZONTAL:
            return float(remote.leftHorizontal) / STICK_MAX_VALUE;
        case Channel::LEFT_VERTICAL:
            return float(remote.leftVertical) / STICK_MAX_VALUE;
    }
    return 0;
}


int16_t Remote::getChannelInt(Channel ch) const
{
    switch (ch)
    {
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

void Remote::printAxisData() const{
    printf("%d %d %d %d\n", getChannelInt(Remote::Channel::LEFT_HORIZONTAL), getChannelInt(Remote::Channel::LEFT_VERTICAL), getChannelInt(Remote::Channel::RIGHT_HORIZONTAL), getChannelInt(Remote::Channel::RIGHT_VERTICAL));
}

Remote::SwitchState Remote::getSwitch(Switch sw) const
{
    switch (sw)
    {
        case Switch::LEFT_SWITCH:
            return remote.leftSwitch;
        case Switch::RIGHT_SWITCH:
            return remote.rightSwitch;
    }
    return SwitchState::UNKNOWN;
}

int16_t Remote::getMouseX() const { return remote.mouse.x; }

int16_t Remote::getMouseY() const { return remote.mouse.y; }

int16_t Remote::getMouseZ() const { return remote.mouse.z; }

bool Remote::getMouseL() const { return remote.mouse.l; }

bool Remote::getMouseR() const { return remote.mouse.r; }

bool Remote::keyPressed(Key key) const { return (remote.key & (1 << (uint8_t)key)) != 0; }

int16_t Remote::getWheel() const { return remote.wheel; }

void Remote::parseBuffer()
{
    // values implemented by shifting bits across based on the dr16
    // values documentation and code created last year\

    int16_t potentialRX = int16_t((rxBuffer[0] | rxBuffer[1] << 8) & 0x07FF);
    int16_t potentialRY = int16_t((rxBuffer[1] >> 3 | rxBuffer[2] << 5) & 0x07FF);
    int16_t potentialLX = int16_t((rxBuffer[2] >> 6 | rxBuffer[3] << 2 | rxBuffer[4] << 10) & 0x07FF);
    int16_t potentialLY = int16_t((rxBuffer[4] >> 1 | rxBuffer[5] << 7) & 0x07FF);

    if(
        potentialRX-1024 < -660 || potentialRX-1024 > 660 ||
        potentialRY-1024 < -660 || potentialRY-1024 > 660 ||
        potentialLX-1024 < -660 || potentialLX-1024 > 660 ||
        potentialLY-1024 < -660 || potentialLY-1024 > 660 || 0){
        
        badDataChainNumber += 1;
        goodDataChainNumber = 0;
    }else{
        goodDataChainNumber += 1;
        badDataChainNumber = 0;

        remote.rightHorizontal = potentialRX;
        remote.rightVertical = potentialRY;
        remote.leftHorizontal = potentialLX;
        remote.leftVertical = potentialLY;

        remote.rightHorizontal -= 1024;
        remote.rightVertical -= 1024;
        remote.leftHorizontal -= 1024;
        remote.leftVertical -= 1024;

        remote.leftSwitch = SwitchState(((rxBuffer[5] >> 4) & 0x000C) >> 2);
        remote.rightSwitch = SwitchState(((rxBuffer[5] >> 4) & 0x0003));

        remote.mouse.x = int16_t((rxBuffer[6]) | (rxBuffer[7] << 8));
        remote.mouse.y = int16_t((rxBuffer[8]) | (rxBuffer[9] << 8));
        remote.mouse.z = int16_t((rxBuffer[10]) | (rxBuffer[11] << 8));

        remote.mouse.l = rxBuffer[12];
        remote.mouse.r = rxBuffer[13];

        remote.key = (rxBuffer[14]);
    }

    

    //printf("%d \t", (rxBuffer[14] >> 1 | rxBuffer[15] << 7) & 0x07FF);
    //printf("\n");
    //printf("%d\t %d\t %d\t %d\t", remote.leftHorizontal, remote.leftVertical, remote.rightHorizontal, remote.rightVertical);
    //printf("\n");
    // the first 6 bytes refer to the remote channel values

    // switches on the dji remote - their input is registered
    switch (((rxBuffer[5] >> 4) & 0x000C) >> 2)
    {
        case 1:
            remote.leftSwitch = SwitchState::UP;
            break;
        case 3:
            remote.leftSwitch = SwitchState::MID;
            break;
        case 2:
            remote.leftSwitch = SwitchState::DOWN;
            break;
        default:
            remote.leftSwitch = SwitchState::UNKNOWN;
            break;
    }

    switch ((rxBuffer[5] >> 4) & 0x003)
    {
        case 1:
            remote.rightSwitch = SwitchState::UP;
            break;
        case 3:
            remote.rightSwitch = SwitchState::MID;
            break;
        case 2:
            remote.rightSwitch = SwitchState::DOWN;
            break;
        default:
            remote.rightSwitch = SwitchState::UNKNOWN;
            break;
    }

    // remaining 12 bytes (based on the DBUS_BUF_LEN variable
    // being 18) use mouse and keyboard data
    // 660 is the max value from the remote, so gaining a higher
    // value would be impractical.
    // as such, the method returns null, exiting the method.
    if ((abs(remote.rightHorizontal) > 660) || (abs(remote.rightVertical) > 660) ||
        (abs(remote.leftHorizontal) > 660) || (abs(remote.leftVertical) > 660))
    {
        return;
    }

    // mouse input
    remote.mouse.x = rxBuffer[6] | (rxBuffer[7] << 8);    // x axis
    remote.mouse.y = rxBuffer[8] | (rxBuffer[9] << 8);    // y axis
    remote.mouse.z = rxBuffer[10] | (rxBuffer[11] << 8);  // z axis
    remote.mouse.l = static_cast<bool>(rxBuffer[12]);     // left button click
    remote.mouse.r = static_cast<bool>(rxBuffer[13]);     // right button click

    // keyboard capture
    remote.key = rxBuffer[14] | rxBuffer[15] << 8;
    // Remote wheel
    remote.wheel = (rxBuffer[16] | rxBuffer[17] << 8) - 1024;

    remote.updateCounter++;
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

