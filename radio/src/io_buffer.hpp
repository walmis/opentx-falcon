/*
 * io_buffer.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: walmis
 */

#ifndef IO_BUFFER_HPP_
#define IO_BUFFER_HPP_

#include <inttypes.h>
#include <stddef.h>

class IOBuffer {

public:
	IOBuffer();
	IOBuffer(uint16_t size);
	IOBuffer(uint8_t* buf, size_t size);

	//~IOBuffer();

	bool allocate(uint16_t size) { return _allocBuffer(size); }

	size_t bytes_free();
	size_t bytes_used();

	bool empty() { return head == tail; }

	size_t write(const uint8_t* buffer, size_t size);
	size_t write(uint8_t c);
	int16_t read(void);
	int16_t read(uint8_t* buffer, size_t size);

	void flush(void);

private:
	void _freeBuffer();
	bool _allocBuffer(uint16_t size);

	volatile uint16_t head, tail;	///< head and tail pointers
	uint16_t mask;					///< buffer size mask for pointer wrap
	uint8_t *bytes;					///< pointer to allocated buffer
};

#endif /* IO_BUFFER_HPP_ */
