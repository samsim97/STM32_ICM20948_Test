#pragma once
class String {
public:
	String();
	String(const char* str);
	String(const String& other);
	String& operator=(const String& other);
	~String();

	size_t size() const;

private:
	char* data;
	size_t length;
	size_t capacity;

};