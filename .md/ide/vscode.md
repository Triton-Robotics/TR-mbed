1. Ensure the following VSCode extensions are installed:

![vscode_1.png](../.assets/vscode_1.png)
![vscode_2.png](../.assets/vscode_2.png)

2. Open the project folder. Upon opening the folder, you will have to select the correct
   compiler as shown:

![vscode_3.png](../.assets/vscode_3.png)

3. Optional: Locate the CMake-Tools extension settings using `CTRL + <Comma>`.

Add the following arguments to enable the Ninja build tool to speed up compile
times. 

![vscode_5.png](../.assets/vscode_5.png)
![vscode_6.png](../.assets/vscode_6.png)

> Note :you may have to change the `-j 16` arg to the number of processes your CPU
can handle, which can be found via the `nproc` command.

4. Use `CTRL + SHIFT + P` to open the command palette, then select `C/C++: Edit Configurations (JSON)`.
   Append the following to your `c_cpp_properties.json`

```json
{
    "configurations": [
        {
            "name": "CMake",
            "compileCommands": "${config:cmake.buildDirectory}/compile_commands.json",
            "configurationProvider": "ms-vscode.cmake-tools"
        }
    ],
    "version": 4
}
```

5. Use `CTRL + SHIFT + P` to open the command palette, then select `CMake: Configure`. The
   configure step should only be run once per project, or after editing any `CMakeLists.txt`.

![vscode_7.png](../.assets/vscode_7.png)

6. Open the command palette, then select `CMake: Build Target`.
   You will be prompted to choose a target, as shown below:

![vscode_8.png](../.assets/vscode_8.png)

> I have not found a good VSCode plugin to flash an STM32 board via `OpenOCD`. I encourage VSCode users to append the
necessary instructions to this guide. For now, I recommend flashing the board via the command line as shown
in the [main README](../../README.md#flashing-stm32).