#include "SimpleBuffer.h"

/*------------------------------Constructor Methods------------------------------*/

SimpleBuffer::SimpleBuffer(const int size)
{
	store_ = new char[size]();
	size_ = size;
	front_ = 0;
	back_ = 0;
}

SimpleBuffer::~SimpleBuffer()
{

}

/*------------------------------Public Methods------------------------------*/

bool SimpleBuffer::push(char c)
{
	store_[front_] = c;

	front_++;

	if(front_ >= size_)
	{
		front_ = 0;
	}

	if(front_ == back_)
	{
		back_++;
	}

	if(back_ >= size_)
	{
		back_ = 0;
	}

	return true;
}

char SimpleBuffer::pop()
{
	char c;

	if(available())
	{
		c = store_[back_];

		back_++;

		if(back_ >= size_)
		{
			back_ = 0;
		}
	}
	else
	{
		c = '\0';
	}

	return c;
}

char SimpleBuffer::peek()
{
	return store_[back_];
}

bool SimpleBuffer::available()
{
	if(back_ != front_)
	{
		return true;
	}
	else
	{
		return false;
	}
}