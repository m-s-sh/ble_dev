/*  
 *  mk_trace.h - UART trace library
 *  
 * 	For using mk_trace functionality:
 *	1. include this mk_trace.h file
 *	2. call once mk_trace_init(TX_BUF_SIZE, RX_BUF_SIZE);
 *	Now you are ready to use mk_trace(...) for sending diagnostic messages via UART/USB to PC
 *	Note: TX_BUF_SIZE and RX_BUF_SIZE must be power of 2
 *	
 *  Copyright (c) 2013 Mommosoft Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef __TRACE_UART__
#define	__TRACE_UART__

#ifdef	MK_TRACE_UART
	uint32_t mk_trace_init(uint32_t tx_buf_size, uint32_t rx_buf_size);
	uint32_t mk_trace(const char* format, ...);
#else
	#define	mk_trace_init(...)
	#define mk_trace(...)
#endif


#endif
