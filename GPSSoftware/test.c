#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#define MAXBUFFER 1024

int set_interface_attribs (int fd, int speed, int parity) {

        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block) {

        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

char text[MAXBUFFER];

int charToInt(char *arr, int s, int e){
    int num = 0;
    for(int i = s; i < e+1; i+=1){
        num *= 10;
        num += arr[i]-'0';
    }
    return num;
}

double degMinTODD(int d, int min, int sec){
    return (double)d+(min/60.0)+((sec/1000.0)/3600.0);
}
struct coordinates{
    float x;
    float y;
}first, second, third, vec;

void create_vector(float x, float y){

    first.x = third.x;
    first.y = third.y;
    third.x = second.x;
    third.y = second.y;
    second.x = x;
    second.y = y;
    
    vec.x = second.x - first.x;
    vec.y = second.y - first.y;

    //printf("\n%f %f", vec.x,vec.y);
}

void GPSstring(char *arr){
    int pegepind = 7;
    int pegepind2 = 0;
    int i = 0;
    double north = 0;
    double east = 0;
    int minN = 0, minE = 0;
    int secN = 0, secE = 0;
    int degN = 0, degE = 0;
    
    //if(arr[3] == 'G' && arr[4] == 'L' && arr[5] == 'L'){
    //North spherical coordinate
        while(1){
            if(arr[i] == 'N'){
                minN = charToInt(arr,i-9,i-8);
                secN = charToInt(arr,i-6,i-2);
                if(arr[i-12] == ','){
                    degN = charToInt(arr,i-11,i-10);
                }else{
                    degN = charToInt(arr,i-12,i-10);
                }
                break;
            }
            if(i == 25){
                return;
            }
            i += 1;
        }
    i = 0;
    //East spherical coordinate
    while(1){
        if(arr[i] == 'E'){
            minE = charToInt(arr,i-9,i-8);
            secE = charToInt(arr,i-6,i-2);
            if(arr[i-12] == ','){
                degE = charToInt(arr,i-11,i-10);
            }else{
                degE = charToInt(arr,i-12,i-10);
            }
            break;
        }
        if(i == 45){
            return;
        }
      i += 1;
    }

	
    north = degMinTODD(degN,minN,secN);
    east = degMinTODD(degE,minE,secE);
    printf("\n%f  %f", north, east);
    
    //create_vector(north, east);
    //}else{
        //printf("%s", "Jeg er return");
      //  return;
    //}
    return;
}

char testArr[] = {'$', 'G', 'P', 'G', 'L', 'L', ',', '5', '5', '2', '2','.','5','9','5','5','2',',','N',',','0','1','0','2','3','.','4','9','5','1','6',',','E','1','7','1','9','3','6',',','A'};

int main(){
	char *portname = "/dev/ttyAMA0";

	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf ("error %d opening %s: %s", errno, portname, strerror (errno));
		return 1;
	}

	set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	//write (fd, "hello!\n", 7);           // send 7 character greeting

	//usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
	int i = 0;
	int bufPointer = 0;
	char buffer[2][MAXBUFFER];
	char john = 1;	
	while(1){
	/*	
		int n = read(fd, buffer[bufPointer]+i, MAXBUFFER-i);
		john = 1;
		for(int j = 0 ; j < n ; j++){

			if(buffer[bufPointer][j+i] == '\n'){
				john = 0;
				if(j != n-1){
					memcpy(buffer[!bufPointer], buffer[bufPointer]+(i+j+1), n-j-1);
					buffer[bufPointer][i+j] = '\0';
					i = n-j-1;
				}else{
					buffer[bufPointer][i+j] = '\0';
					i = 0;
				}
				//GPSstring(buffer[bufPointer]);
                GPSstring(testArr);
				//printf("%s\n", buffer[bufPointer]);
				bufPointer != bufPointer;
			}

		}
		if(john){
			i += n;
		}
        //printf("%s\n", buffer[bufPointer]);
	}
    */
        GPSstring(testArr);
    }
	return 0;
}
