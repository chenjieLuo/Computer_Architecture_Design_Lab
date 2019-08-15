      char *prodMessage = "The product is ";
      void print_int(int a)
      {
        asm(	"li $2, 1\n\t"
        "syscall"
        : /*No outputs*/
        : "r"(a)
        : "%v0" );
      }

      void print_string(char *a)  //define the function print_string for later use
      {
        asm(	"li $2, 4\n\t"
        "syscall"
        : /*No outputs*/
        : "r"(a)
        : "%v0" );
      }

      int read_int()		//define the function read_int for later use
      {
        register unsigned long __v0 asm("$2");

        asm(	"li $2, 5\n\t"
        "syscall"
        : /*No outputs*/
        : /*No inputs*/
        : "%v0" );

        return __v0;
      }
