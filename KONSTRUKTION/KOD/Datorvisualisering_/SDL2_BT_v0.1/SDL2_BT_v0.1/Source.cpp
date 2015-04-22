#include <iostream>
#include <SDL.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <windows.h>
#include <stdint.h>


using namespace std;

#define BUFFSIZE 10

HANDLE hComm; //Handle for comport
DCB dcbSerialParams = { 0 }; //dcb for comport
COMMTIMEOUTS timeouts = { 0 }; //value for timeoutconfig

DWORD BytesWritten_ = 0; // how many bytes that were actually written
DWORD BytesRead_ = 0; // how many bytes that were actually read
char ReadBuff_[BUFFSIZE];
char WriteBuff_[BUFFSIZE];


//Initialize commport
void Init_CommPort(string comport){

	hComm = CreateFile(
		comport.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

//	Parametersettings ----------------------------------
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hComm, &dcbSerialParams)){
//		Error getting state
		printf("Error getting state %d\n");
	}

	// Setting baudrate and similar settings
	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	SetCommState(hComm, &dcbSerialParams);

	if (!SetCommState(hComm, &dcbSerialParams)){
		printf("Error setting state. %d \n");

	}

	timeouts.ReadIntervalTimeout = 1;
	timeouts.ReadTotalTimeoutMultiplier = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 50;
	timeouts.WriteTotalTimeoutConstant = 10;

	if (!SetCommTimeouts(hComm, &timeouts))
	{
		printf("Error setting time-outs. %d");
		return;
	}

}
//Initialize welcome
void Init_Welcome()
{
	cout << "\n*******************************************\n\n";
	cout << "          ResQ.Pl - Master Race \n";
	cout << "\n*******************************************\n\n";
	cout << " \n - 'init' to establish connection\n - 'close connection' to close initialized connection \n - 'read' / 'write' to do that\n Enter command:" << endl;
	cout << "> ";
}

void Read_ComData(){
	fill(begin(ReadBuff_), end(ReadBuff_), '\0');
	
	if (!ReadFile(hComm, ReadBuff_, BUFFSIZE, &BytesRead_, NULL)) // BytesRead contains amount of read bytes
	{
		cerr << "Reading from file failed!\n";
	}
	cout << "\nReadbuff: ";

	for (int i = 0; i < BUFFSIZE; i++)
		{
			char c = ReadBuff_[i];
			cout << c;
		}
		cout << "\n Bytes read: " << BytesRead_ << endl << "> ";
	
}

void Write_ComData(){
	fill(begin(WriteBuff_), end(WriteBuff_), '\0');
	char W_in{};
	int i = 0;
	while ((cin.peek() != '\n') && (i != BUFFSIZE)){
		cin >> W_in;
		WriteBuff_[i] = W_in;
		i++;
	}
	cout << "\nWriteBuff_: " << WriteBuff_ << endl;
	if (!WriteFile(hComm, WriteBuff_, BUFFSIZE, &BytesWritten_, NULL)) // "100" says how many bytes to write
	{
		cerr << "Writing to file failed!\n";
	}
	else
	{
		cin >> W_in;
		cout << "Data written\n> ";
	}
}



//Handles input from keyboard
void Input_Loop(){
	string instring_;
	char in;
	bool ComInit_ = false;
	cin >> noskipws;
	while (cin.peek() != EOF)
	{
		cin >> in;

		if (in == '\n'){ //if command has been entered
			// init com-procedure if stringmatch
			if (ComInit_ == false && instring_ == "init"){
				Init_CommPort("COM3"); //Initializes COM3
				ComInit_ = true;
				cout << "COM3" << " initialized!\n> ";
			}

			// initialization already done
			else if (ComInit_ == true && instring_ == "init"){
				cout << "COM3 already initialized!\n> ";
			}

			//Write to com
			else if (instring_ == "write"){
				cout << "Write what to send:\n> ";
				Write_ComData();
			}

			//Read from com
			else if (instring_ == "read")	{
				cout << "Reading...\n";
				Read_ComData();
			}

			// Close connection
			else if ((instring_ == "close connection") | (instring_ == "close"))
			{
				CloseHandle(hComm);
				ComInit_ = false;
				cout << "Connection to COM3 closed!\n> ";
			}
			else
			{
				cout << "Unknown input\n> ";
			}

			instring_ = "";
		}
		else
		{
			instring_ = instring_ + in;
		}
	}
}


int main(int argc, char* argv[])
{
	Init_Welcome();
	Input_Loop();
	return 0;
}

//
//using namespace std;
//
//HANDLE hComm; //Handle for comport
//DCB dcbSerialParams = { 0 }; //dcb for comport
//COMMTIMEOUTS timeouts = { 0 }; //value for timeoutconfig
//DWORD BytesWritten_ = 0; // how many bytes that were actually written
//
////Struct for linked list
//typedef struct node { // definition of the linked list node
//	char val;
//	struct node *next;
//} buffer_; 
//
//buffer_ *head_BTout = NULL;
//buffer_ *head_BTin = NULL;
//
////Add node  to linked lsit
//void add_node(buffer_* lst_head, char val) 
//{
//	if (lst_head != NULL)
//	{
//		buffer_ * curr = lst_head;
//		while (curr->next != NULL){ // step to end of list
//			curr = curr->next;
//		}
//		curr->next = new buffer_;
//		curr->next->val = val;
//		curr->next->next = NULL; // Add node last.
//	}
//	else
//	{
//		lst_head = new buffer_;
//		lst_head->val = val;
//		lst_head->next = NULL;
//	}
//} 
////Pop node linked list
//int pop_node(buffer_ ** lst_head)
//{
//	buffer_* next_node = NULL;
//	char retval = 0;
//
//	if (*lst_head != NULL)
//	{
//		next_node = (*lst_head)->next;
//		retval = (*lst_head)->val;
//		free(*lst_head);
//		*lst_head = next_node;
//	}
//
//	return 0;
//} //
////Flush linked list
//void flush_list(buffer_ ** lst_head)
//{
//	while (*lst_head != NULL)
//	{
//		pop_node(lst_head);
//	}
//}
////Initialize commport
//void Init_CommPort(string comport){
//	
//	hComm = CreateFile(
//		comport.c_str(),
//		GENERIC_READ | GENERIC_WRITE,
//		0,
//		0,
//		OPEN_EXISTING,
//		FILE_ATTRIBUTE_NORMAL,
//		0);
//
//	//Parametersettings ----------------------------------
//		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
//		if (!GetCommState(hComm, &dcbSerialParams)){
//			//Error getting state
//			printf("Error getting state %d\n");
//		}
//	
//		//Setting baudrate and similar settings
//		dcbSerialParams.BaudRate = CBR_115200;
//		dcbSerialParams.ByteSize = 8;
//		dcbSerialParams.StopBits = ONESTOPBIT;
//		dcbSerialParams.Parity = NOPARITY;
//		
//		SetCommState(hComm, &dcbSerialParams);
//
//		if (!SetCommState(hComm, &dcbSerialParams)){
//			printf("Error setting state. %d \n");
//
//		}
//
//		//SetCommMask(hComm, EV_RXCHAR | EV_TXEMPTY);
//		//
//		//memset(&ov_, 0, sizeof(ov_)); //Mem alloc for ov_
//		//
//		//ov_.hEvent = CreateEvent(0, 1, 0, 0); //Default security, start as non-signaled.
//		//if (ov_.hEvent == NULL){
//		//	printf("CreateEvent failed (%d)\n", GetLastError());
//		//	return;	
//		//}
//		//HANDLE arrHandles[2]; 
//		//
//		//DWORD events = GetCommMask(hComm, &events);
//		//cout << events;
//
//		timeouts.ReadIntervalTimeout = MAXDWORD;
//		timeouts.ReadTotalTimeoutMultiplier = 50;
//		timeouts.ReadTotalTimeoutConstant = 50;
//		timeouts.WriteTotalTimeoutMultiplier = 50;
//		timeouts.WriteTotalTimeoutConstant = 10;
//
//		if (!SetCommTimeouts(hComm, &timeouts))
//		{
//			printf("Error setting time-outs. %d");
//			return;
//		}
//
//}
////Initialize welcome
//void Init_Welcome()
//{
//	cout << "\n*******************************************\n\n";
//	cout << "          ResQ.Pl - Master Race \n";
//	cout << "\n*******************************************\n\n";
//	cout << " \n - 'init' to establish connection\n - 'close connection' to close initialized connection \n - 'read' / 'write' to do that\n Enter command:" << endl;
//	cout << "> ";
//}
////Reads data from comport
//void Read_ComData(){
//	char indata;
//	int8_t test = 1;
//	DWORD numb_readbytes;
//	head_BTin = NULL;
//
//	do //Read from hComm until it's empty, aka numb_readbytes = 0
//	{
//		if (!ReadFile(hComm, &indata, 1, &numb_readbytes, NULL)){
//			printf("Reading from file failed!%d\n");
//		}
//		cout << "indata: " << indata << endl;
//		add_node(head_BTin, indata);
//		cout << numb_readbytes;
//	} while (numb_readbytes > 0);
//	
//	cout << "Data read: ";
//	int i = 0;
//	while (head_BTin != NULL){
//		cout << i << endl;
//		cout << pop_node(&head_BTin);
//		i++;
//	}
//	cout << "\n> ";
//}
//
////Writes data to comport
//void Write_ComData(){
//	char in_;
//	DWORD numb_writtenbytes;
//	while (cin.peek() != '\n'){
//		cin >> in_;
//		add_node(head_BTout, in_);
//		WriteFile(hComm, &in_, 1, &numb_writtenbytes, NULL);
//		cout << "Written bytes: " << numb_writtenbytes << endl;
//	}
//
//	cout << "Writebuff: ";
//	while (head_BTout->next != NULL){
//		cout << " poopade: ";
//		cout << pop_node(&head_BTout);
//	}
//	cin >> in_;
//	cout << "\n> ";
//}
////Handles input from keyboard
//void Input_Loop(){
//	string instring_;
//	char in;
//	bool ComInit_ = false;
//	cin >> noskipws;
//	while (cin.peek() != EOF)
//	{
//		cin >> in;
//
//		if (in == '\n'){ //if command has been entered
//
//			// init com-procedure if stringmatch
//			if (ComInit_ == false && instring_ == "init"){
//				Init_CommPort("COM3"); //Initializes COM3
//				ComInit_ = true;
//				cout << "COM3" << " initialized!\n> ";
//			}
//
//			// initialization already done
//			else if (ComInit_ == true && instring_ == "init"){
//				cout << "COM3 already initialized!\n> ";
//			}
//
//			//Write to com
//			else if (instring_ == "write"){
//				cout << "Write what to send:\n> ";
//				Write_ComData();
//			}
//
//			//Read from com
//			else if (instring_ == "read")	{
//				cout << "Reading...\n";
//				Read_ComData();
//			}
//
//			// Close connection
//			else if ((instring_ == "close connection") | (instring_ == "close"))
//			{
//				CloseHandle(hComm);
//				ComInit_ = false;
//				cout << "Connection to COM3 closed!\n> ";
//			}
//			else
//			{
//				cout << "Unknown input\n> ";
//			}
//
//			instring_ = "";
//		}
//		else
//		{
//			instring_ = instring_ + in;
//		}
//	}
//}
//
//int main(int argc, char* argv[])
//{
//	//Intro to print out
//	bool ComInit_ = false; //Flag 
//	head_BTout = new buffer_;
//	head_BTout->val = 0;
//	head_BTout->next = NULL;
//	head_BTin = new buffer_;
//	head_BTin->val = 0;
//	head_BTin->next = NULL;
//	Init_Welcome();
//	Input_Loop();
//
//	//Read in from keyboard and initialize connection
//	delete head_BTin;
//	delete head_BTout;
//	return 0;
//}
//
//////Variables for initiating com-port
////HANDLE hSerial;
////DCB dcbSerialParams = { 0 };
////COMMTIMEOUTS timeouts = { 0 };
////
//////For reading/writning from com
////int8_t ReadBuff_[10]; // Buffer to store indata from com, [2]-bytes
////DWORD BytesRead_ = 0; // how many bytes that were actually read
////int8_t WriteBuff_[10]; // Buffer to store outdata from com, [2]-bytes
////
//
////void Read_ComData() //Reads data from hSerial (COM4) and stores in ReadBuff.
////{
////	fill(begin(ReadBuff_), end(ReadBuff_), '\0');
////	if (!ReadFile(hSerial, ReadBuff_, 10, &BytesRead_, NULL)) // BytesRead contains amount of read bytes
////	{
////		cerr << "Reading from file failed!\n";
////	}
////	cout << "\nReadbuff: ";
////	for (int i = 0; i < 10; i++)
////	{
////		char c = static_cast<char>(ReadBuff_[i]);
////		cout << c;
////	}
////	cout << "\n Bytes read: " << BytesRead_ << endl << "> ";
////
////}
////
//
////void Write_ComData() //Writes data to hSerial (COM4) and stores in WriteBuff.
////{
////	fill(begin(WriteBuff_), end(WriteBuff_), '\0');
////	char W_in{};
////	int i = 0;
////	while (cin.peek() != '\n'){
////		cin >> W_in;
////		WriteBuff_[i] = W_in;
////		i++;
////	}
////
////	cout << "\nWriteBuff_: " << WriteBuff_ << endl;
////	if (!WriteFile(hSerial, WriteBuff_, 10, &BytesWritten_, NULL)) // "100" says how many bytes to write
////	{
////		cerr << "Writing to file failed!\n";
////	}
////	else
////	{
////		cin >> W_in;
////		cout << "Data written\n> ";
////	}
////
////}
////
////