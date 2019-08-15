extern char *prodMessage;
      void print_int(int a);
      void print_string(char *a);
      int read_int();

      int my_mul(int a, int b)   //operate the multiplication
      {
        int i, ret = 0;
        for(i=0; i<b; i++)
        ret = ret + a;
        return ret;
      }

      int main(void)
      {
        print_string("Enter the first number");  //print out the string
        int num1 = read_int();                   //read input from user
        print_string("Enter the second number");  //print out the string
        int num2 = read_int();			//read input from user
        print_string(prodMessage);
        print_int(my_mul(num1, num2));		//print out the result after operating the function my_mul
        return 0;
      }
