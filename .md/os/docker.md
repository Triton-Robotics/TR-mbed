### Usage

Use the included `docker-compose.yml` to automate environment setup and building.

This command will setup a full environment then build the TR-TestBench target.
```
docker compose run testbench
```

Output can be found in the `build` directory for flashing via `openocd`. 

The current `docker-compose.yml` contains rules for TestBench and Infantry. Feel free to add more
as necessary.
