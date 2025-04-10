# Why is this here
This document is here to point you in the direction you desire.

## If you want to review the details and results of this project
please reference the Final Project PDF document and ignore the rest of this README.

## If you want to know how to run the project and analyze the outputs 
keep reading.

## How to run the code
*The code works by just copy pasting the code from the included `main.c` into the existing lab2 project. Further details are below*

1. Download STMCubeIDE and STM32CubeProgrammer.
2. Extract the file `Final Project Outputs Part 1.zip`.
3. Extract the file `Part1_Project.zip`.
4. Upload `a440_16-bit.bin` to the STM32 dev board using STMCubeProgrammer. We uploaded this data to 0x08020000 as per previous lab recommendations. This is the input data that will be filtered.
5. Navigate to inside of lab2_proj and run .project file.
6. At the top of `main.c`, uncomment the macro corresponding to the function you wish to test.
7. Enter the block size you wish to process. Note: this has been validated for 3 and 16 sample block size. It should work for any reasonable sample size, but your mileage could vary.
8. Run the program with your favourite compiler and debug settings with a clock frequency of 96MHz in debug and note the memory location of `outputBuffer[]`.
9. Extract the binary data with STMCubeProgrammer.

## How to verify the data
1. Start up the `Binary_Processing.ipynb` file in your favourite place to run Jupyter Notebooks.
2. Set the prgram to import your output binary from the previous section.
3. Run the code and view your beautiful new sine wave.