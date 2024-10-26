This setup guide uses Homebrew to install the required packages. If not already installed,
Homebrew can be installed from the [official website](https://brew.sh/).

### Base development packages
```shell
brew install cmake
brew install python
brew install --cask gcc-arm-embedded
brew install openocd
brew install ninja
```

### mbed-tools & Dependencies
```shell
pip3 install  \
mbed-tools    \
prettytable   \
future        \
jinja2        \
intelhex
```

### Enable mbed-tools via command line
```shell
echo "export PATH=\"`python3 -m site --user-base`/bin:\$PATH\"" >> ~/.bashrc

source ~/.bashrc
```

> Note: May have to replace  `~/.bashrc` with your shell (eg. `~/.zshrc` for zsh).

> Note: If you are getting a missing arm-none-eabi-gcc, you may have to run `brew install armmbed/formulae/arm-none-eabi-gcc`

### Return to [main setup guide](../../README.md#setup) to continue setup.
