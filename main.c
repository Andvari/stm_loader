/*
 * main.c
 *
 *  Created on: Mar 19, 2013
 *      Author: nemo
 */
/*
 * 	SATOR
 *  AREPO
 *  TENET
 *  OPERA
 *  ROTAS
 */
#include "stdio.h"
#include "string.h"
#include "fcntl.h"
#include "unistd.h"
#include "termios.h"
#include "sys/stat.h"

#define	CMD_INIT				((unsigned char)0x7F)
#define	CMD_GET					((unsigned char)0x00)
#define	CMD_GET_BL_VER			((unsigned char)0x01)

#define	CMD_GET_ID				((unsigned char)0x02)
#define	CMD_READ_MEM			((unsigned char)0x11)
#define	CMD_GO					((unsigned char)0x21)
#define	CMD_WRITE_MEM			((unsigned char)0x31)
#define	CMD_ERASE				((unsigned char)0x43)
#define	CMD_EXT_ERASE			((unsigned char)0x44)
#define	CMD_WRITE_PROTECT		((unsigned char)0x63)
#define	CMD_WRITE_UNPROTECT		((unsigned char)0x73)
#define	CMD_READOUT_PROTECT		((unsigned char)0x82)
#define	CMD_READOUT_UNPROTECT	((unsigned char)0x92)

#define	CMD_GLOBAL_ERASE		((unsigned char)0xFF)

#define	ACK		0x79
#define	NACK	0x1F

#define	MAX_BYTES_TO_READ	256

int fd;
unsigned char errcode;

char get_command(void);

unsigned char send_command(char *, unsigned char);
void send_byte(unsigned char);
char recv_byte(unsigned char *);

void read_mem(unsigned char *, unsigned int addr, unsigned int num);
char write_mem(unsigned char *, unsigned int addr, unsigned int num);

char is_ack(void);

void write_image(char *);
int decode_string(unsigned char *, int);
char flash_it(unsigned char *, unsigned int, int);

int main(int argc, char *argv[]){
	int fil;
	int i;
	unsigned char bytes_to_read;
	unsigned char str[MAX_BYTES_TO_READ];
	unsigned char bootloader_version;
	unsigned char rpd;
	unsigned char rpe;

	unsigned int addr;
	unsigned int num;
	unsigned int remain;
	unsigned char checksum;

	unsigned char command;
	struct termios options;
	int idx_file = -1;
	int idx_tty = -1;
	char tty[32] = "/dev/ttyUSB0";

	for(i=1; i<argc; i++){
		if(strncmp(argv[i], "/dev/", 5) == 0){
			strcpy(tty, argv[i]);
		}
		else{
			if((fd = open(argv[i], O_RDONLY)) != -1){
				idx_file = i;
			}
		}
	}

	if(idx_tty != -1){
		printf("Trying %s...", argv[idx_tty]);
		fd = open(argv[idx_tty], O_RDWR);
	}
	else{
		printf("Trying /dev/ttyUSB0...");
		fd = open("/dev/ttyUSB0", O_RDWR);
	}

	if(fd == -1){
		printf("ERROR\n");
		return (1);
	}
	else{
		printf("OK\n");
	}

	fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);

	cfsetospeed(&options, B115200);

	options.c_cflag |= PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= CS8;
	options.c_lflag &= ~ICANON;

	options.c_cc[VMIN]  = 1;
	options.c_cc[VTIME] = 0;

	tcsetattr(fd, TCSANOW, &options);

	if(idx_file != -1){
		printf("File to be written...%s\n", argv[idx_file]);
		write_image(argv[idx_file]);
		close(fd);
		return (0);
	}

	while((command = get_command()) != 0){
		switch(command){
/* INIT */
			case 1:		send_command("INIT", CMD_INIT);
						break;
/* GET */
			case 2:		if(send_command("GET", CMD_GET) == ACK){
							printf("Bytes to read...");
							recv_byte(&bytes_to_read);
							printf("%d\n", bytes_to_read);

							printf("Reading bytes...");
							for(i=0; i<=bytes_to_read; i++)recv_byte(&str[i]);

							if(is_ack()){
								for(i=0; i<=bytes_to_read; i++) printf("%x ", str[i]);
								printf("\n");
							}
						}
						break;
/* GET VERSION */
			case 3:		if(send_command("GET VER", CMD_GET_BL_VER) == ACK){
							printf("Params reading...");
							recv_byte(&bootloader_version);
							recv_byte(&rpd);
							recv_byte(&rpe);
							if(is_ack()){
								printf("%x %x %x\n", bootloader_version, rpd, rpe);
							}
						}
						break;
/* GET ID */
			case 4:		if(send_command("GET ID", CMD_GET_ID) == ACK){
							printf("Bytes to read...");
							recv_byte(&bytes_to_read);
							printf("%d\n", bytes_to_read);

							printf("Reading bytes...");
							for(i=0 ;i<=bytes_to_read; i++){
								recv_byte(&str[i]);
							}

							if(is_ack()){
								for(i=0; i<=bytes_to_read; i++) printf("%x ", str[i]);
								printf("\n");
							}
						}
						break;
/* READ MEM */
			case 5:		printf("Enter start address (HEX)...");
						scanf("%x", &addr);
						printf("Enter num bytes (DEC)...");
						scanf("%d", &num);

						fil = open("./readfile.txt", O_RDWR | O_CREAT, S_IRWXO | S_IRWXU | S_IRWXG);

						for(i=0; i<num/MAX_BYTES_TO_READ; i++){
							read_mem(str, addr + i*MAX_BYTES_TO_READ, MAX_BYTES_TO_READ);
							write(fil, str, MAX_BYTES_TO_READ);
						}

						remain = num%MAX_BYTES_TO_READ;

						if(remain > 0){
							read_mem(str, addr + i*MAX_BYTES_TO_READ, remain);
							write(fil, str, remain);
						}

						close(fil);
						break;
/* GO */
			case 6:		if(send_command("GO", CMD_GO) == ACK){
							printf("Enter start address (HEX)...");
							scanf("%x", &addr);

							printf("Sending start address (%08x)...", addr);
							send_byte((addr >> 24) & 0xFF);	checksum  =	(addr >> 24) & 0xFF;
							send_byte((addr >> 16) & 0xFF);	checksum ^=	(addr >> 16) & 0xFF;
							send_byte((addr >>  8) & 0xFF);	checksum ^=	(addr >>  8) & 0xFF;
							send_byte((addr >>  0) & 0xFF);	checksum ^=	(addr >>  0) & 0xFF;
							send_byte(checksum);
							if(is_ack()){
								printf("Getting acknowledge...");
								is_ack();
							}
						}
						break;
/* WRITE MEM */
			case 7:		printf("Enter start address (HEX)...");
						scanf("%x", &addr);

						fil = open("./writefile.txt", O_RDONLY);
						num = lseek(fil, 0, SEEK_END);
						close(fil);
						fil = open("./writefile.txt", O_RDONLY);

						for(i=0; i<num/MAX_BYTES_TO_READ; i++){
							read(fil, str, MAX_BYTES_TO_READ);
							write_mem(str, addr + i*MAX_BYTES_TO_READ, MAX_BYTES_TO_READ);
						}

						remain = num%MAX_BYTES_TO_READ;

						if(remain%4 != 0) remain += 4-(remain%4);

						if(num%MAX_BYTES_TO_READ > 0){
							read(fil, str, num%MAX_BYTES_TO_READ);
							for(i=0; i<remain-num%MAX_BYTES_TO_READ; i++){
								str[num%MAX_BYTES_TO_READ + i] = 0xFF;
							}
							write_mem(str, addr + i*MAX_BYTES_TO_READ, remain);
						}

						close(fil);
						break;
/* ERASE */
			case 8:		if(send_command("ERASE", CMD_ERASE) == ACK){
							printf("Enter amount of pages to be erased (DEC)...");
							scanf("%d", &num);

							if(num == 255){
								send_command("GLOBAL ERASE", CMD_GLOBAL_ERASE);
							}
							else{
								if(num > 0){
									for(i=0; i<num; i++){
										printf("Enter [%d] page number to be erased...", i);
										scanf("%d", &addr);
										str[i] = addr&0xFF;
									}

									printf("Erasing pages...");
									send_byte(num-1);			checksum  =	num-1;
									for(i=0; i<num; i++){
										send_byte(str[i]);		checksum ^= str[i];
									}
									send_byte(checksum);
									is_ack();
								}
								else{
									printf("Nothing to erase\n");
								}
							}
						}
						break;
/* WRITE PROTECT */
			case 9:		if(send_command("WRITE PROTECT", CMD_WRITE_PROTECT) == ACK){
							printf("Enter amount of sectors to be protected (DEC)...");
							scanf("%d", &num);

							if(num > 0){
								for(i=0; i<num; i++){
									printf("Enter [%d] sector number to be protected...", i+1);
									scanf("%d", &addr);
									str[i] = addr&0xFF;
								}

								printf("Protecting sectors...");
								send_byte(num-1);			checksum  =	num-1;
								for(i=0; i<num; i++){
									send_byte(str[i]);		checksum ^= str[i];
								}
								send_byte(checksum);
								if(is_ack()){
									sleep(1);
									send_command("INIT", CMD_INIT);
								}
							}
							else{
								printf("Nothing to protect\n");
							}
						}
						break;
/* WRITE UNPROTECT */
			case 10:	if(send_command("WRITE UNPROTECT", CMD_WRITE_UNPROTECT) == ACK){
							printf("Unprotecting memory for writing...");
							if(is_ack()){
								sleep(1);
								send_command("INIT", CMD_INIT);
							}
						}
						break;
/* READOUT PROTECT */
			case 11:	if(send_command("READOUT PROTECT", CMD_READOUT_PROTECT) == ACK){
							printf("Protecting memory for readout...");
							if(is_ack()){
								sleep(1);
								send_command("INIT", CMD_INIT);
							}
						}
						break;
/* READOUT UNPROTECT */
			case 12:	if(send_command("READOUT UNPROTECT", CMD_READOUT_UNPROTECT) == ACK){
							printf("Unprotecting memory for readout...");
							if(is_ack()){
								sleep(1);
								send_command("INIT", CMD_INIT);
							}
						}
						break;

			default:	printf("Not implemented yet\n");
						break;
		}
	}
	close(fd);

	printf("Bye.\n");
	return (0);
}

void read_mem(unsigned char *str, unsigned int fromaddr, unsigned int totalnum){
	int i, j;
	unsigned char checksum;

	union{
		unsigned int	ui;
		unsigned char	c[4];
	}addr;

	union{
		unsigned int	ui;
		unsigned char	c[4];
	}num;

	addr.ui = fromaddr;
	num.ui	= totalnum - 1;

	if(send_command("READ MEM", CMD_READ_MEM) == ACK){

		printf("Sending start address (%08x)...", addr.ui);
		send_byte(addr.c[3]);	checksum  = addr.c[3];
		send_byte(addr.c[2]);	checksum ^= addr.c[2];
		send_byte(addr.c[1]);	checksum ^= addr.c[1];
		send_byte(addr.c[0]);	checksum ^= addr.c[0];
		send_byte(checksum);
		if(is_ack()){
			printf("Sending num bytes (%d)...", num.c[0]);
			checksum = 0xFF;
			send_byte(num.c[0]);	checksum ^= num.c[0];
			send_byte(checksum);

			if(is_ack()){
				i=0;
				while(i<=num.c[0]){
					for(j=0; j<16; j++, i++){
						if(i <= num.c[0]){
							recv_byte(&str[i]);
							//printf("%02x ", str[i]);
						}
						else{
							break;
						}
					}
					//printf("\n");
				}
			}
		}
	}
}

char write_mem(unsigned char *str, unsigned int fromaddr, unsigned int totalnum){
	int i, j;
	char checksum;

	union{
		unsigned int	ui;
		unsigned char	c[4];
	}addr;
	union{
		unsigned int	ui;
		unsigned char	c[4];
	}num;

	addr.ui = fromaddr;
	num.ui	= totalnum-1;

	if(send_command("WRITE MEM", CMD_WRITE_MEM) == ACK){
		printf("Sending start address (%08x)...", addr.ui);
		send_byte(addr.c[3]);	checksum  = addr.c[3];
		send_byte(addr.c[2]);	checksum ^= addr.c[2];
		send_byte(addr.c[1]);	checksum ^= addr.c[1];
		send_byte(addr.c[0]);	checksum ^= addr.c[0];
		send_byte(checksum);
		if(is_ack()){

			printf("Sending num bytes (%d) and data packet...", num.c[0]);
			checksum = 0x0;
			send_byte(num.c[0]);	checksum ^= num.c[0];

			i=0;
			while(i<=num.c[0]){
				for(j=0; j<16; j++, i++){
					if(i <= num.c[0]){
						send_byte(str[i]);	checksum ^= str[i];
					}
				}
			}
			send_byte(checksum);
			is_ack();
		}
	}
	return checksum;
}

void send_byte(unsigned char byte){
	write(fd, &byte, 1);
	tcdrain(fd);
}

unsigned char send_command(char *title, unsigned char byte){
	printf("Sending %s command...", title);

	send_byte(byte);
	if(byte != 0x7F) send_byte(byte^0xFF);

	if(is_ack()){
		return (ACK);
	}

	return(NACK);
}

char recv_byte(unsigned char *byte){
	read(fd, byte, 1);
	return (byte[0]);
}

char is_ack(void){
	if(recv_byte(&errcode) == ACK){
		printf("ACK\n");
		return (1);
	}
	else{
		printf("NACK\n");
		return (0);
	}
}

char get_command(void){
	int a;
	printf("\n\n");
	printf("0 - quit              1 - init              2 - get               3 - get version\n");
	printf("4 - get id            5 - read memory       6 - go                7 - write memory\n");
	printf("8 - erase             9 - write protect     10- write unprotect   11- read protect\n");
	printf("12- read unprotect\n");

	printf("Select command: ");
	scanf("%d", &a);

	return (a);
}

#define MAX_BYTES_IN_RECORD	255

#define	TYPE_DATA		0
#define	TYPE_EOF		1
#define	TYPE_ADDR		2
#define	TYPE_EXT_ADDR	4

typedef struct{
	unsigned char header;
	unsigned char num_bytes;
	unsigned int base_addr;
	unsigned short int addr_offset;
	unsigned char type;
	unsigned char data[MAX_BYTES_IN_RECORD];
	unsigned char checksum;
}RECORD;

void write_image(char *filename){
	int i;
	int fil;
	unsigned int is_addr_set = 0;
	unsigned int eof = 0;
	unsigned char data_to_stm[MAX_BYTES_IN_RECORD];
	char write_status = 0;
	unsigned char tmp[4];
	unsigned int start_address;
	unsigned int current_lenght;

	RECORD rec;

	fil = open(filename, O_RDONLY);

	if (fil == -1){
		printf("Error opening file\n");
		return;
	}

	if(send_command("INIT", CMD_INIT) != ACK)return;

	current_lenght = 0;
	while(!eof){

		do{
			if(read(fil, &rec.header, 1) == 0) eof = 1;
			if(eof)break;
		}
		while(rec.header != ':');

		if(eof)break;

		if(read(fil, &tmp, 2) == 0) eof = 1;
		rec.num_bytes = (char)decode_string(&tmp[0], 2);
		if(eof)break;

		if(read(fil, &tmp, 4) == 0) eof = 1;
		rec.addr_offset = (unsigned short)decode_string(&tmp[0], 4);
		if(eof)break;

		if(read(fil, &tmp[0], 2) == 0) eof = 1;
		rec.type = (char)decode_string(&tmp[0], 2);
		if(eof)break;

		for(i=0; i<rec.num_bytes; i++){
			if(read(fil, &tmp[0], 2) == 0) eof = 1;
			rec.data[i] = (char)decode_string(&tmp[0], 2);
			if(eof)break;
		}

		if(read(fil, &tmp[0], 2) == 0) eof = 1;
		rec.checksum = (char)decode_string(&tmp[0], 2);
		if(eof)break;

		switch(rec.type){
		case	TYPE_DATA:		if(is_addr_set > 0){
									if(is_addr_set == 1){
										start_address = rec.base_addr + rec.addr_offset;
										is_addr_set = 2;
									}
									if(start_address + current_lenght == rec.base_addr + rec.addr_offset){
										if(current_lenght + rec.num_bytes <= MAX_BYTES_TO_READ){
											memcpy(&data_to_stm[current_lenght], rec.data, rec.num_bytes);
											current_lenght += rec.num_bytes;
										}
										else{
											write_status = flash_it(data_to_stm, start_address, current_lenght);
											if(write_status!=0)eof=1;
											start_address = rec.base_addr + rec.addr_offset;
											memcpy(data_to_stm, rec.data, rec.num_bytes);
											current_lenght = rec.num_bytes;
										}
									}
									else{
										write_status = flash_it(data_to_stm, start_address, current_lenght);
										if(write_status!=0)eof=1;
										start_address = rec.base_addr + rec.addr_offset;
										memcpy(data_to_stm, rec.data, rec.num_bytes);
										current_lenght = rec.num_bytes;
									}
								}
								else{
									printf("Data record without base address\n");
									write_status = 1;
									eof = 1;
								}
								break;
		case	TYPE_EOF:		eof = 1;
								break;
		case	TYPE_ADDR:		write_status = flash_it(data_to_stm, start_address, current_lenght);
								if(write_status!=0)eof=1;
								current_lenght = 0;
								rec.base_addr = ((rec.data[0] * 256 + rec.data[1]) * 16) << 16;
								is_addr_set = 1;
								break;
		case	TYPE_EXT_ADDR:	write_status = flash_it(data_to_stm, start_address, current_lenght);
								if(write_status!=0)eof=1;
								current_lenght = 0;
								rec.base_addr = (rec.data[0] * 256 + rec.data[1]) << 16;
								is_addr_set = 1;
								break;
		default:
								break;
		}
	}

	if(write_status == 0){
		printf("File wrote OK\n");
	}
	else{
		printf("File not written\n");
	}


	close(fil);
	return;
}

int decode_string(unsigned char *str, int len){
	int i;
	int a;
	a = 0;
	for(i=0; i<len; i++){
		if(str[i] > 57){
			a = a*16 + str[i] - 65 + 10;
		}
		else{
			a = a*16 + str[i] - 48;
		}
	}

	return a;
}

char flash_it(unsigned char *data, unsigned int addr, int size){
	int i;
	int write_status = 0;
	int num_bytes_to_stm;
	unsigned char data_from_stm[MAX_BYTES_IN_RECORD];
	unsigned char checksum_to_stm;
	unsigned char checksum_from_stm;

	if(size > 0){
		num_bytes_to_stm = size;

		if(num_bytes_to_stm%4 != 0) num_bytes_to_stm += 4-(num_bytes_to_stm%4);
		for(i=0; i<num_bytes_to_stm - size; i++){
			data[size + i] = 0xFF;
		}

		checksum_to_stm = write_mem(data, addr, num_bytes_to_stm);
		read_mem(data_from_stm, addr, num_bytes_to_stm);
		checksum_from_stm = num_bytes_to_stm-1;
		for(i=0; i<num_bytes_to_stm; i++){
			checksum_from_stm ^= data_from_stm[i];
		}

		if(checksum_to_stm != checksum_from_stm) write_status = 1;
	}
	return write_status;
}
