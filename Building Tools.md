
### Compiler
Just as humans successfully communicate in language understandable to both talker and listener, A computer needs **binary language** to understand what a human demands. A binary language comprises 0s and 1s. So a compiler is a program that converts programming language into binary language for human-computer successful interaction. We will be using clang as a compiler.\

#### ### Clang As Compiler

The Clang tool is a front end compiler that is used to compile programming languages such as C++, C, Objective C++ and Objective C into machine code

## The Compilation Process

####  Preprocess - A preprocessor is responsible for **adding the contents of the header files** in the main code.
A header file contain definitions of functions and variables, which can be used into any C++ program by using the pre-processor `#include` statement. The header file being used here is `iostream`. It provides basic input and output services for C++ programs. So, a preprocessor is responsible for removing comments, expanding macros (if any) and expanding included files. 


> [!NOTE]
> clang++ hello.cpp - This command produces no output, but should produce an executable file.
> 
> The default output file name for executables created by clang is **a.out**, when no output name is specified. In order to run that file, you have to write **./** in the beginning.
> 
> clang++ -E hello.cpp > hello.i  this command will give you no output if it worked


#### Compile - The main role of a compiler is to translate the preprocessed code to assembly code. These low level instructions are specific to a particular processor architecture.

