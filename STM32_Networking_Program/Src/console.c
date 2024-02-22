/**
 ******************************************************************************
 * @file	: console.c
 * @brief	: This file contains the implementation of the console module, which
 *            provides a command-line interface for interacting with the network
 *            functionality of the system. It includes functions for initializing
 *            the console, prompting the user for commands, and executing the
 *            corresponding actions based on the user's input.
 * @details	: The console module allows the user to transmit and receive messages,
 *            set the sender and receiver addresses, and perform other network-related
 *            operations. It also provides a help command to display a list of available
 *            commands and their descriptions.
 ******************************************************************************/

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

// Library

#include <stdio.h>
#include <string.h>

// Project
#include "F446RE.h"
#include "network.h"
#include "uart_driver.h"
#include "delay.h"

/*
 ******************************************************************************
 * Variables
 ******************************************************************************
 */

static char str[258];
static char * token1;
static char * token2;

/*
 ******************************************************************************
 * Function Prototypes
 ******************************************************************************
 */

static int addr_usr_find(char* user);
static char* usr_addr_find(int addr);

/*
 ******************************************************************************
 * Function Definitions
 ******************************************************************************
 */

/**
 * @brief
 *
 */
void console_init(void)
{

	// Initialize UART connection
	init_usart2(921600, F_CPU);

}

/**
 * @brief Displays a user prompt and executes commands based on user input.
 *
 * This function prompts the user with "net> " and waits for user input. It then parses the input
 * and executes the corresponding command. The supported commands are:
 * - "tx [string]": Transmits the provided string to the specified recipient.
 * - "btx [string]": Broadcasts the provided string to all recipients.
 * - "rx": Prints the latest received message to the console.
 * - "usr [username]": Sets the sender's address to the specified username.
 * - "recip [username]": Sets the recipient's address to the specified username.
 * - "r": Enters RECEIVER TEST MODE, continuously printing received messages to the console.
 * - "h": Displays a help message with a list of available commands.
 *
 * @note This function assumes that the necessary variables and functions are defined and implemented
 *       elsewhere in the code.
 */
void user_prompt(void)
{
	printf("net> ");
	fgets(str, 258, stdin);
	token1 = strtok(str," \n\r");
#ifdef DE_CONSOLE
	printf("token1 = '%s'\n\r", token1);
#endif
	token2 = strtok(NULL,"\n\r");
#ifdef DE_CONSOLE
	if(token2)
	{
		printf("token2 = '%s'\n\r", token2);
	}
#endif
	if(!strcmp(token1,"tx"))
	{
		if(token2)
		{
			if(get_sender())
			{
				if(get_reciever())
				{
					// encode packet and transmit
					encode(token2);
					printf("transmitting '%s'...\n\r",token2);
				}
				else
				{
					printf("Recipient not set\n\r");
				}
			}
			else
			{
				printf("User not set\n\r");
			}
		}
		else
		{
			printf("Need string to transmit\n\r");
		}
	}
	else if(!strcmp(token1,"btx"))
	{
		if(token2)
		{
			int temp = get_reciever();
			set_reciever(0xFF);
			encode(token2);
			printf("broadcasting '%s'...\n\r",token2);
			set_reciever(temp);
		}
		else
		{
			printf("Need string to broadcast\n\r");
		}
	}
	else if(!strcmp(token1,"rx"))
	{
		// Get Ascii Data
		parse_packet();

		// Checks for a received message, prints it to console, then returns to command prompt
		if(new_message_flag())
		{

			printf("%s\n\r", get_ascii_data());
#ifdef DE_NET_RX
			printf("%d\n\r", get_dataSize());
			int* data = get_raw_data();
			printf("0b");
			for(int i = 0; i < get_dataSize(); i++)
			{
				printf("%i", data[i]);
			}
			printf("\n\r");

			// Reset recived data for more data
			reset_rx_data();
#endif
		}
		else
		{
			printf("No new messages\n\r");
		}
	}
	else if(!strcmp(token1,"usr"))
	{
		int addr = addr_usr_find(token2);
		if(addr != 0x00)
		{
			set_sender(addr);
		}
		else
		{
			printf("Invalid user name\n\r");
		}
	}
	else if(!strcmp(token1,"recip"))
	{
		int addr = addr_usr_find(token2);
		if(addr != 0x00)
		{
			set_reciever(addr);
		}
		else
		{
			printf("Invalid user name\n\r");
		}
	}
	else if(!strcmp(token1,"r"))
	{
		printf("Console is now in RECEIVER TEST MODE\n\r");
		for(;;)
		{
			// If there's data inside the buffer, then print it
			if(new_message_flag())
			{
				printf("%s\n\r", get_ascii_data());
			}
		}
	}
	else if(!strcmp(token1,"crc"))
	{
		if(!strcasecmp(token2,"on"))
		{
			set_reciever(1);
		}
		else if(!strcasecmp(token2,"off"))
		{
			set_reciever(0);
		}
	}
	else if(!strcmp(token1,"h"))
	{
		printf("=========================================================\n\r");
		printf("tx [string]\n\r");
		printf("	Transmits the string input by the user to the\n\r");
		printf("    recipient\n\r");
		printf("\n\r");
		printf("btx\n\r");
		printf("    Transmits the ");
		printf("rx\n\r");
		printf("	Prints the latest message to the console\n\r");
		printf("\n\r");
		printf("r\n\r");
		printf("	Continuously prints messages to the console\n\r");
		printf("	(Does not return)\n\r");
		printf("usr\n\r");
		printf("    Sets the address of the network interface based on\n\r");
		printf("    the name input\n\r");
		printf("recip\n\r");
		printf("    Sets the message recipient based on the name input\n\r");
		printf("\n\r");
		printf("=========================================================\n\r");
	}
	else
	{
		printf("ERR: unknown command.\n\r");
	}
}

/**
 * @brief Decodes a user's name to an address
 *
 * @param user - User's name
 * @return - User's address
 */
int addr_usr_find(char* user)
{
	if(!strcmp(user,"Bayan"))
	{
		return 0x10;
	}
	else if(!strcmp(user,"Xiaohui"))
	{
		return 0x11;
	}
	else if(!strcmp(user,"Jacobi"))
	{
		return 0x12;
	}
	else if(!strcmp(user,"Caleb"))
	{
		return 0x14;
	}
	else if(!strcmp(user,"Paige"))
	{
		return 0x15;
	}
	else if(!strcmp(user,"Dan"))
	{
		return 0x18;
	}
	else if(!strcmp(user,"Scott"))
	{
		return 0x19;
	}
	else if(!strcmp(user,"Olivia"))
	{
		return 0x1C;
	}
	else if(!strcmp(user,"Luis"))
	{
		return 0x1D;
	}
	else if(!strcmp(user,"Collin"))
	{
		return 0x20;
	}
	else if(!strcmp(user,"Jeremiah"))
	{
		return 0x21;
	}
	else if(!strcmp(user,"Kali"))
	{
		return 0x24;
	}
	else if(!strcmp(user,"Jake"))
	{
		return 0x25;
	}
	else if(!strcmp(user,"Asher"))
	{
		return 0x26;
	}
	else if(!strcmp(user,"JP"))
	{
		return 0x28;
	}
	else if(!strcmp(user,"Erik"))
	{
		return 0x29;
	}
	else if(!strcmp(user,"Dylan"))
	{
		return 0x2A;
	}
	else if(!strcmp(user,"Roland"))
	{
		return 0x2C;
	}
	else if(!strcmp(user,"Cody"))
	{
		return 0x2D;
	}
	else if(!strcmp(user,"Alex"))
	{
		return 0x2E;
	}
	else if(!strcmp(user,"Josh"))
	{
		return 0x30;
	}
	else if(!strcmp(user,"Russel"))
	{
		return 0x31;
	}
	else if(!strcmp(user,"Matt M"))
	{
		return 0x32;
	}
	else if(!strcmp(user,"Tyler"))
	{
		return 0x34;
	}
	else if(!strcmp(user,"Brayden"))
	{
		return 0x35;
	}
	else if(!strcmp(user,"Ryan"))
	{
		return 0x38;
	}
	else if(!strcmp(user,"Michael"))
	{
		return 0x39;
	}
	else if(!strcmp(user,"Charles"))
	{
		return 0x3A;
	}
	else if(!strcmp(user,"Troy"))
	{
		return 0x3C;
	}
	else if(!strcmp(user,"Priya"))
	{
		return 0x3D;
	}
	else if(!strcmp(user,"Matt H"))
	{
		return 0x3E;
	}
	else if(!strcmp(user,"Jack"))
	{
		return 0x40;
	}
	else if(!strcmp(user,"Zach"))
	{
		return 0x41;
	}
	else if(!strcmp(user,"Daniel"))
	{
		return 0x42;
	}
	else
	{
		return 0x00;
	}
}

/**
 * @brief Decode a user's address to a name
 *
 * @param addr - User's address
 * @return - User's name
 */
char* usr_addr_find(int addr)
{
	switch(addr)
	{
	case 0x00:
		return "Invalid";
	case 0x01:
		return "Hub";
	case 0x10:
		return "Bayan";
	case 0x11:
		return "Xiaohui";
	case 0x12:
		return "Jacobi";
	case 0x14:
		return "Caleb";
	case 0x15:
		return "Paige";
	case 0x18:
		return "Dan";
	case 0x19:
		return "Scott";
	case 0x1C:
		return "Olivia";
	case 0x1D:
		return "Luis";
	case 0x20:
		return "Collin";
	case 0x21:
		return "Jeremiah";
	case 0x24:
		return "Kali";
	case 0x25:
		return "Jake";
	case 0x26:
		return "Asher";
	case 0x28:
		return "JP";
	case 0x29:
		return "Erik";
	case 0x2A:
		return "Dylan";
	case 0x2C:
		return "Roland";
	case 0x2D:
		return "Cody";
	case 0x2E:
		return "Alex";
	case 0x30:
		return "Josh";
	case 0x31:
		return "Russel";
	case 0x32:
		return "Matt M";
	case 0x34:
		return "Tyler";
	case 0x35:
		return "Brayden";
	case 0x38:
		return "Ryan";
	case 0x39:
		return "Michael";
	case 0x3A:
		return "Charles";
	case 0x3C:
		return "Troy";
	case 0x3D:
		return "Priya";
	case 0x3E:
		return "Matt H";
	case 0x40:
		return "Jack";
	case 0x41:
		return "Zach";
	case 0x42:
		return "Daniel";
	case 0xFF:
		return "Broadcast";
	default:
		return "Invalid";
	}
}
