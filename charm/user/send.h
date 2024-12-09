/*
 * send.h
 *
 *  Created on: 2024 Nov 22
 *      Author: zaqc
 */

#ifndef USER_SEND_H_
#define USER_SEND_H_

#define	PKT_MAGIC_ID	0xC647F16A
#define PKT_DATA_COUNT	2048

extern volatile uint8_t spi_tx_rdy;

void pkt_envelop(uint16_t *in_data, uint16_t *out_data, uint16_t len, uint8_t pack_size);
int16_t pkt_pack(uint16_t *in_data, uint16_t *out_data, uint16_t len);
void pkt_send(uint16_t *data, uint16_t len);

#endif /* USER_SEND_H_ */
