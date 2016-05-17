/*
 * io_buffer.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: walmis
 */

#include "io_buffer.hpp"
#include <string.h>
#include <stdio.h>

#define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))

size_t IOBuffer::bytes_free() {
	return ((mask) - ((head - tail) & mask));
}

size_t IOBuffer::bytes_used() {
	return ((head - tail) & mask);
}
IOBuffer::IOBuffer() :
	head(0), tail(0), mask(0), bytes(0)
{
}
IOBuffer::IOBuffer(uint16_t size) :
	head(0), tail(0), mask(0), bytes(0)
{
	_allocBuffer(size);
}

IOBuffer::IOBuffer(uint8_t* buf, size_t size) :
			head(0), tail(0), mask(size-1), bytes(buf)
{
	uint16_t shift;
	// Compute the power of 2 greater or equal to the requested buffer size
	// and then a mask to simplify wrapping operations.  Using __builtin_clz
	// would seem to make sense, but it uses a 256(!) byte table.
	// Note that we ignore requests for more than BUFFER_MAX space.
	for (shift = 1; (1U << shift) < size; shift++);

	mask = (1U << shift) - 1;
}

//IOBuffer::~IOBuffer() {
//	_freeBuffer();
//}

int16_t IOBuffer::read(uint8_t* buffer, size_t size) {
	size = MIN(size, bytes_used());
	if(!size) return 0;

	size_t bytes_read1 = MIN(size, (size_t)(mask + 1) - tail);

	memcpy(buffer, bytes + tail, bytes_read1);
	memcpy(buffer + bytes_read1, bytes, size - bytes_read1);

	tail = (tail + size) & mask;

	return size;
}

size_t IOBuffer::write(const uint8_t* buffer, size_t size) {
	size_t space = bytes_free();
	if (space <= 0) {
		return 0;
	}
	if (size > (size_t) (space)) {
		// throw away remainder if too much data
		size = space;
	}
	if (tail > head) {
		// perform as single memcpy
		memcpy(&bytes[head], buffer, size);
		head = (head + size) & mask;
		return size;
	}
	// perform as two memcpy calls
	uint16_t n = (mask + 1) - head;
	if (n > size)
		n = size;

	memcpy(&bytes[head], buffer, n);
	head = (head + n) & mask;
	buffer += n;
	n = size - n;
	if (n > 0) {
		memcpy(&bytes[0], buffer, n);
		head = (head + n) & mask;
	}
	return size;
}

size_t IOBuffer::write(uint8_t c) {
	uint16_t i = (head + 1) & mask;

	// if there isn't enough room for it in the transmit buffer
	if (i == tail) {
		return 0;
	}
	// add byte to the buffer
	bytes[head] = c;
	head = i;
	// return number of bytes written (always 1)
	return 1;
}

int16_t IOBuffer::read(void) {
	uint16_t c;
	// if the head and tail are equal, the buffer is empty
	if ((head == tail))
		return (-1);

	// pull character from tail
	c = bytes[tail];
	tail = (tail + 1) & mask;
	return (c);
}

void IOBuffer::flush(void) {
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of _rxBuffer->head but before writing
	// the value to _rxBuffer->tail; the previous value of head
	// may be written to tail, making it appear as if the buffer
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of head but before writing
	// the value to tail; the previous value of rx_buffer_head
	// may be written to tail, making it appear as if the buffer
	// were full, not empty.
	head = tail;
	// don't reverse this or there may be problems if the TX interrupt
	// occurs after reading the value of _txBuffer->tail but before writing
	// the value to _txBuffer->head.
	tail = head;
}

void IOBuffer::_freeBuffer() {
	head = tail = 0;
	mask = 0;
	if (0 != bytes) {
		delete[] bytes;
		bytes = 0;
	}
}

bool IOBuffer::_allocBuffer(uint16_t size) {
	if(!size) return false;
	uint16_t mask;
	uint16_t shift;
	// init buffer state
	head = tail = 0;
	// Compute the power of 2 greater or equal to the requested buffer size
	// and then a mask to simplify wrapping operations.  Using __builtin_clz
	// would seem to make sense, but it uses a 256(!) byte table.
	// Note that we ignore requests for more than BUFFER_MAX space.

	for (shift = 1; (1U << shift) < size; shift++)
		;

	mask = (1U << shift) - 1;
	// If the descriptor already has a buffer allocated we need to take
	// care of it.
	if (bytes) {
		// If the allocated buffer is already the correct size then
		// we have nothing to do
		if (this->mask == mask)
			return true;

		// Dispose of the old buffer.
		delete[] bytes;
	}
	this->mask = mask;
	// allocate memory for the buffer - if this fails, we fail.
	bytes = new uint8_t[mask + (size_t)(1)];
	return (bytes != 0);
}
