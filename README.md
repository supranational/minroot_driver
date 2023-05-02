# Minroot ASIC Driver

The minroot ASIC is a joint project sponsored by the Ethereum Foundation, Protocol Labs, and the Filecoin Foundation. The goal is to explore verifiable delay functions (VDF) as a secure and provable source of randomness. This repository provides a basic driver and example program for interfacing with the prototype minroot ASIC devices.

## Setup

The build process should work on Linux and Mac

```
./build.sh
```

We also supply a prebuilt universal binary for Mac under github actions. Click on Actions, select the latest workflow run and download the 'universal-minroot' artifact.

For Linux, if gmp is not already installed:
```
wget https://gmplib.org/download/gmp/gmp-6.2.1.tar.lz
tar --lzip -xf gmp-6.2.1.tar.lz
cd gmp-6.2.1/
./configure --enable-cxx
make -j2
make check
sudo make install
cd ..
```

## Run

```
./minroot
```

Available command line options
```
  -s <seed>
  -n <num_jobs>
  -e <num_engines> (1 - 12)
  -f <frequency> (should be between 200 and 1100)
  -v <voltage> (should be between 0.70 and 1.0)
  -i <iterations>
```

## Performance

The minroot ASIC can perform approximately 4000 iterations/second/MHz. Therefore at 1 GHz the expected performance is on the order of 4M iterations/second. A high frequency CPU can do approximately 400K iterations/second, which makes the ASIC ~10x faster.

## TODO
- Add engine mask
- Add timing data

## License
The blst library is licensed under the [Apache License Version 2.0](LICENSE) software license.
