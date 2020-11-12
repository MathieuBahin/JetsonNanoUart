
/************************************/
/* @auteur Mathieu Bahin            */
/* @date_création mars 2020         */
/* @version 1.0                     */
/* @email bahin.mathieu@gmail.com   */
/************************************/

#ifndef _UART_H
#define _UART_H


// Define Constants
const char *uart_target = "/dev/ttyTHS1";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       B115200


class Uart {
private:
  /* data */
  int fid;
public:
  unsigned char serial_message[NSERIAL_CHAR];


  Uart ();
  void sendUart(unsigned char *msg);
  bool sendUart_fb(unsigned char *msg);
  void readUart();
  void closeUart();

};
#endif
