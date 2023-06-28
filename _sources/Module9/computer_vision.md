# Lab 2

### Objectives
- Students will learn about registers, RAM, flash ROM, and the stack.
- Students will learn debugging techniques like single stepping, breakpoints, and watch windows.

### Overview
In this lab students will write a program in assembly that determines whether integer numbers are prime or not.

### Deliverables
**Deliverable 1:** 
* [25 Points] Push your code to your repository. You must add comments to your code. **It is your responsibility to check your files have been successfully pushed to your Bitbucket repository.**
* [25 Points] Provide a screen shot of the following and submit via Gradescope.  Use **Snip \& Sketch** (Win+Shift+S) in Windows 10 or Shift+CMD+4 in Mac to take a screenshot.  Do NOT take a picture of computer screen with your phone because it will introduce sampling aliasing (more details in ECE215/ECE315). 

- [10 Points] The addresses of each of the variables stored in RAM and ROM (use the Memory Browser and the pointers you created. You can search the Memory Browser by variable name or memory address).
- [15 Points] The contents of `Res` after execution of the code (use the Memory Browser and change the format to "8-Bit Hex - TI Style"). If you click on Resume (F8) followed by Suspend (Alt+F8), it will stop at `Exit B Exit`. Then, you can browse the memory.
- [-30 Points] Take a picture of your screen with a mobile device or digital camera and upload it to Gradescope. Yes, I am serious...

### Setup
- Connect the LaunchPad to your computer via the provided USB cable.
- Open Code Composer Studio (CCS) and select your workspace.
- Ensure your Project Explorer is open on the left of the CCS screen. 
- Select View > Project Explorer.
- Open the Lab02\_PrimeNumbers project by double clicking it.



## Complete Lab02\_IsPrime.asm 

 An equivalent C program is given by
```C 
if (n == 1) {
    return 0;   // 1 is not a prime number.
}
// We need to check if n is divisible by 2,3,...,n/2.
int m = n >> 1; // m = n/2, integer division 
for (int i = 2; i <= m; ++i) {
     if (n % i == 0) { // n is divisible by i
         return 0;  // n is not a prime number
     }
 }
return 1; // n is a prime number
```

**Variables in RAM:**
- `Res` - allocate 16 bytes of memory to store the results. If the first number is prime, you need to set the first byte of Res to 1. Otherwise, set it to 0.


**Variables in ROM:**
- `Nums` - integers to test.
- `ResAddr` - pointer to Res.
- `NumsAddr` - pointer to Nums.

**Main:**
Comment out the main function (line 58) in Homework02\_main.asm and uncomment the main function (line 58) in Lab02\_IsPrime.asm.

**Modulo:** For the modulo operation, use the assembly code provided below.  You can change the registers to fit in your code. 

```asm
UDIV  R2, R0, R1 ; R2 = R0/R1 (n/i)
MUL   R3, R2, R1 ; R3 = R2*R1 (int(n/i)*i)
CMP   R0, R3     ; n == int(n/i)*i ?,  n is divisible by i if R0 == R3.
```


