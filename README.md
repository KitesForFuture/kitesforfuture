# kitesforfuture
Kites for Future revolutionises the way we produce electricity

# Setup

Follow the instructions [here](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/#get-started-connect) to set up the ESP IDF.

Afterwards, you can compile ESP-IDF projects.
Go to the project directory and run:

```bash
    idf.py build
```

To set up the serial port on your computer for communicating with the ESP, just run:

```bash
    idf.py menuconfig
```

Flash the compiled code with

```bash
    idf.py -p (PORT) flash
```

where `PORT` is the serial port configured above.