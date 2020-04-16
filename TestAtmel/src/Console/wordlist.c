 // names are kept in flash; arrays are used; reduces space requirements
#include "cli.h"

#define NAMES(name) const char name[] = {
#define NAME(s) s "\000"
#define END_NAMES ""}; // empty string to cover empty array

#define NONAMES(name) const char name[] = {""};
#define NOBODIES(functions) const struct constantCall functions[] = {{NULL}};

#define BODIES(functions) const vector functions[] = {
#define CBODIES const struct constantCall constantbodies[] = {
#define BODY(f) (vector)f,
#define CONSTANTBODY(f)  { cii, &f },
#define CONSTANTNUMBER(n)  { cii, (Byte *)n },
#define END_BODIES };

void cii(void);


// Words
NAMES(wordnames)
	NAME("uptime")		//  print running time in seconds
//	setdate setTimeDate // ( Y M D h m s day ) set date and time and day (of week)
//	date printTimeDate // print date and time and day of week
//    viewcal viewCalibrateClock // view calibration of high freq. RC oscillator
//    syncinterval setTimeSync // (n) set time sync interval for n seconds
//    timeinterrupts cliTimeInterrupts // (f) set to true to enable interrupt timing
//    testudm testUserDataMemory // test the user data memory
//    quote testPostQuote // <["]string["]> test post quote function; quotes optional
//	pp printPins // print all the pins with names, levels, mode
	NAME("te")		//  print counter, compare value and list of actions and due dates
	NAME("pa")		//  print actions in queue
	NAME("tn")		//  dump out names in action name arraye
	NAME("mstats")		//  list stats for machines
	NAME("0stats")		//  initialize stats to zero
	NAME("testtime")		//  ( s ) test ticks, timeouts and time for s seconds
	NAME("tickms")		//  ( tick - ms ) convert tick to milliseconds
	NAME("gtt")		//  print tick and time
	NAME("freq")		//  get high frequency clock in MHz
	NAME("help")		//  print words with one line help; allow wild card filtering; parenthesis show ( args - results ) and precede the command; angle brackets show arguments that follow commands
	NAME("words")		//  list all words in dictionary
	NAME("dup")		//  ( n - n n ) make a copy of the top data stack item
	NAME("drop")		//  ( n - ) throw away the top data stack item
	NAME("swap")		//  ( n m - m n ) swap top two items on the data stack
	NAME("over")		//  ( n m - n m n ) copy 2nd data stack item to top of data stack
	NAME("?dup")		//  ( n - n n | - 0 ) duplicate top data stack item if not 0
	NAME("sp!")		//  ( ... - ) empty the data stack
	NAME(">r")		//  ( n - ) (R - n ) push the top item of the data stack onto the return stack
	NAME("r")		//  ( - n ) (R n - n ) copy the top item of the return stack onto the data stack
	NAME("r>")		//   ( - n ) (R n - ) move top item on return stack to data stack
	NAME("and")		//  ( n m - p ) bitwise AND top two data stack items and leave on top
	NAME("or")		//   ( n m - p ) bitwise OR top two data stack items and leave on top
	NAME("xor")		//  ( n m - p ) bitwise XOR top two data stack items and leave on top
	NAME("not")		//  ( n - n' ) invert all bits on the top data stack item
	NAME("negate")		//  ( n - -n ) two's complement of top data stack item
	NAME("abs")		//  ( n - |n|) top data stack item is made positive
	NAME("shift")		//  ( n m - p ) shift n by m bit left for minus and right for positive
	NAME("+")		//  ( n m - p ) add top two data stack items and leave on top
	NAME("-")		//  ( n m - p ) subtract top data stack item from next item and leave on top
	NAME("*")		//  ( n m - p ) multiply next data stack item by top and leave on top
	NAME("/")		//  ( n m - q ) divide next data stack item by top and leave on top
	NAME("mod")		//  ( n m - r ) modulus next data stack item by top and leave on top
	NAME("/mod")		//  ( n m - q r ) return divide and modulus from top item into next item
	NAME("=")		//  ( n m - f ) leave a boolean on stack after equating top two data stack items
	NAME("<")		//  ( n m - f ) leave a boolean on stack indicating if next is less than top
	NAME(">")		//  ( n m - f ) leave a boolean on stack indicating if next is greater than top
	NAME("min")		//   ( n m - n|m) leave minimum of top two stack items
	NAME("max")		//  ( n m - n|m) leave maximum of top two stack items
	NAME("execute")		//  ( a - ) use the top data stack item as a function call
	NAME("@")		//  ( a - n ) return contents of memory using top stack item as the address (processor sized)
	NAME("!")		//  ( n a - ) store next into memory using top as address (processor sized)
	NAME("c@")		//  ( a - c ) return contents of memory using top stack item as the address (8 bit)
	NAME("c!")		//  ( c a - ) store next into memory using top as address (8 bit)
	NAME("s@")		//  ( a - h ) return contents of memory using top stack item as the address (16 bit)
	NAME("s!")		//  ( h a - ) store next into memory using top as address (16 bit)
	NAME("l@")		//  ( a - n )return contents of memory using top stack item as the address (32 bit)
	NAME("l!")		//  (n a - )store next into memory using top as address (processor sized)
	NAME("+b")		//  ( b a - ) turn on b bits at address a: 0b10001 em +b
	NAME("-b")		//  ( b a - ) turn off b bits at address a: 0b10001 em -b
	NAME("cmove")		//  ( s d n - ) move n bytes from s to d
	NAME("fill")		//  ( s n x - )fill n bytes from s with x
	NAME("erase")		//  ( s n - ) erase n bytes from s
	NAME("here")		//  ( - a ) return address of end of dictionary
	NAME("allot")		//  ( n - ) reserve n bytes after end of dictionary
	NAME("c,")		//  ( c - ) allocate and 1 byte and put value in it
	NAME(",")		//  ( n -s ) allocate 1 cell and put n into it
	NAME("emit")		//  ( c - ) send c to output device
	NAME("strlen")		//  ( a - c ) return length of a string
	NAME("cr")		//  send end of line to output device
	NAME("type")		//  ( a n - ) output n characters starting at a
	NAME("hex")		//  interpret all following numbers as hex
	NAME("decimal")		//  interpret all subsequent numbers as decimal
	NAME("bin")		//  switch to binary numbers
	NAME("oct")		//  switch to octal numbers
	NAME("hold")		//  ( c - ) hold a character in number sequence
	NAME("<#")		//  inititiate a number sequence
	NAME("#")		//  ( n - n' ) convert a digit from n
	NAME("#s")		//  ( n - 0 ) convert all digits in n
	NAME("sign")		//  ( n - ) prepend sign to number sequence if n is negative
	NAME("#>")		//  ( n - a c ) finish number sequence and return address and count
	NAME(".")		//  ( n - ) print n in current number base
	NAME(".r")		//  ( m n - ) print m in right field of n digits
	NAME(".b")		//  ( n - ) print number in binary
	NAME(".d")		//  ( n - ) print number in decimal
	NAME(".h")		//  ( n - ) print number in hex
	NAME(".s")		//  print number of items on data stack and items
	NAME("dump")		//  ( a n - ) dump n 16-byte rows of memory starting at address a
	NAME("resetcli")		//  reset cli including removing all macros
	NAME("words")		//  list all words in dictionary
	NAME(":")		//  start a macro definition
	NAME("constant")		//  ( n - ) give n a name
	NAME("variable")		//  ( n - ) give n a place to be stored at a name
	NAME("]")		//  enter macro mode
END_NAMES

void printUptime(void);
//	setdate setTimeDate // ( Y M D h m s day ) set date and time and day (of week)
//	date printTimeDate // print date and time and day of week
//    viewcal viewCalibrateClock // view calibration of high freq. RC oscillator
//    syncinterval setTimeSync // (n) set time sync interval for n seconds
//    timeinterrupts cliTimeInterrupts // (f) set to true to enable interrupt timing
//    testudm testUserDataMemory // test the user data memory
//    quote testPostQuote // <["]string["]> test post quote function; quotes optional
//	pp printPins // print all the pins with names, levels, mode
void print_te(void);
void print_actions(void);
void dumpTeaNames(void);
void machineStats(void);
void zeroMachineTimes(void);
void test_time(void);
void ticks_ms(void);
void get_tick_time(void);
void get_clock_frequency(void);
void help(void);
void words(void);
void dup(void);
void drop(void);
void swap(void);
void over(void);
void questionDup(void);
void spStore(void);
void tor(void);
void rat(void);
void rfrom(void);
void andOp(void);
void orOp(void);
void xorOp(void);
void notOp(void);
void negateOp(void);
void absOp(void);
void shiftOp(void);
void plusOp(void);
void minusOp(void);
void starOp(void);
void slashOp(void);
void modOp(void);
void slashModOp(void);
void equals(void);
void lessThan(void);
void greaterThan(void);
void minOp(void);
void maxOp(void);
void execute(void);
void fetch(void);
void store(void);
void byteFetch(void);
void byteStore(void);
void shortFetch(void);
void shortStore(void);
void longFetch(void);
void longStore(void);
void plusBits(void);
void minusBits(void);
void byteMove(void);
void byteFill(void);
void byteErase(void);
void here(void);
void cliAllot(void);
void cComma(void);
void comma(void);
void emitOp(void);
void stringLength(void);
void cursorReturn(void);
void type(void);
void hex(void);
void decimal(void);
void bin(void);
void oct(void);
void hold(void);
void startNumberConversion(void);
void convertDigit(void);
void convertNumber(void);
void sign(void);
void endNumberConversion(void);
void dot(void);
void dotr(void);
void dotb(void);
void dotd(void);
void doth(void);
void dots(void);
void dump(void);
void resetCli(void);
void words(void);
void colon(void);
void constant(void);
void variable(void);
void righBracket(void);

BODIES(wordbodies)
	BODY(printUptime)
//	setdate setTimeDate // ( Y M D h m s day ) set date and time and day (of week)
//	date printTimeDate // print date and time and day of week
//    viewcal viewCalibrateClock // view calibration of high freq. RC oscillator
//    syncinterval setTimeSync // (n) set time sync interval for n seconds
//    timeinterrupts cliTimeInterrupts // (f) set to true to enable interrupt timing
//    testudm testUserDataMemory // test the user data memory
//    quote testPostQuote // <["]string["]> test post quote function; quotes optional
//	pp printPins // print all the pins with names, levels, mode
	BODY(print_te)
	BODY(print_actions)
	BODY(dumpTeaNames)
	BODY(machineStats)
	BODY(zeroMachineTimes)
	BODY(test_time)
	BODY(ticks_ms)
	BODY(get_tick_time)
	BODY(get_clock_frequency)
	BODY(help)
	BODY(words)
	BODY(dup)
	BODY(drop)
	BODY(swap)
	BODY(over)
	BODY(questionDup)
	BODY(spStore)
	BODY(tor)
	BODY(rat)
	BODY(rfrom)
	BODY(andOp)
	BODY(orOp)
	BODY(xorOp)
	BODY(notOp)
	BODY(negateOp)
	BODY(absOp)
	BODY(shiftOp)
	BODY(plusOp)
	BODY(minusOp)
	BODY(starOp)
	BODY(slashOp)
	BODY(modOp)
	BODY(slashModOp)
	BODY(equals)
	BODY(lessThan)
	BODY(greaterThan)
	BODY(minOp)
	BODY(maxOp)
	BODY(execute)
	BODY(fetch)
	BODY(store)
	BODY(byteFetch)
	BODY(byteStore)
	BODY(shortFetch)
	BODY(shortStore)
	BODY(longFetch)
	BODY(longStore)
	BODY(plusBits)
	BODY(minusBits)
	BODY(byteMove)
	BODY(byteFill)
	BODY(byteErase)
	BODY(here)
	BODY(cliAllot)
	BODY(cComma)
	BODY(comma)
	BODY(emitOp)
	BODY(stringLength)
	BODY(cursorReturn)
	BODY(type)
	BODY(hex)
	BODY(decimal)
	BODY(bin)
	BODY(oct)
	BODY(hold)
	BODY(startNumberConversion)
	BODY(convertDigit)
	BODY(convertNumber)
	BODY(sign)
	BODY(endNumberConversion)
	BODY(dot)
	BODY(dotr)
	BODY(dotb)
	BODY(dotd)
	BODY(doth)
	BODY(dots)
	BODY(dump)
	BODY(resetCli)
	BODY(words)
	BODY(colon)
	BODY(constant)
	BODY(variable)
	BODY(righBracket)
END_BODIES

// Immediates
NAMES(immediatenames)
	NAME("[")		//  exit macro mode
	NAME("(")		//  start of comment till end of line or )
	NAME("if")		//  ( n - ) execute following code if top of stack is non-zero
	NAME("else")		//  otherwise part of an if statement
	NAME("endif")		//  end of else or if statement
	NAME("begin")		//  start of a loop construct
	NAME("again")		//  end of a continuous loop construct
	NAME("while")		//  ( n - ) conditional choice in a loop construct
	NAME("repeat")		//  go back to the begin part
	NAME("until")		//  ( n - ) go back to the begin statement if stack is zero
	NAME("for")		//  ( n - ) start of a loop which runs n times
	NAME("next")		//  end of a for loop
	NAME("exit")		//  exit macro
	NAME(";")		//  end a macro
END_NAMES

void leftBracket(void);
void comment(void);
void compileIf(void);
void compileElse(void);
void compileEndif(void);
void compileBegin(void);
void compileAgain(void);
void compileWhile(void);
void compileRepeat(void);
void compileUntil(void);
void compileFor(void);
void compileNext(void);
void compileExit(void);
void semiColon(void);

BODIES(immediatebodies)
	BODY(leftBracket)
	BODY(comment)
	BODY(compileIf)
	BODY(compileElse)
	BODY(compileEndif)
	BODY(compileBegin)
	BODY(compileAgain)
	BODY(compileWhile)
	BODY(compileRepeat)
	BODY(compileUntil)
	BODY(compileFor)
	BODY(compileNext)
	BODY(compileExit)
	BODY(semiColon)
END_BODIES

// Constants
NAMES(constantnames)
END_NAMES

NOBODIES(constantbodies)
