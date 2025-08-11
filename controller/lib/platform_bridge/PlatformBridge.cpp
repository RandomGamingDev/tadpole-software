#include "PlatformBridge.h"

#if defined(TARGET_TEENSY41)


#elif defined(TARGET_NATIVE)

std::array<DigitalPin, UINT8_MAX> digital_pins = {};
std::array<AnalogPin, UINT8_MAX> analog_pins = {};

void pinMode(uint8_t pin, uint8_t mode) {
	if (mode != INPUT && mode != OUTPUT && mode != INPUT_PULLUP)
		//throw std::invalid_argument(std::format("0x{:x} is an undefined mode!", digital_pin.mode));
		throw std::invalid_argument("undefined mode!");

	DigitalPin& digital_pin = digital_pins[pin];

	digital_pin.mode = mode;
	if (mode == INPUT_PULLUP)
		digital_pin.pullup = true;
}

void digitalWrite(uint8_t pin, uint8_t val) {
	DigitalPin& digital_pin = digital_pins[pin];

	switch (digital_pin.mode) {
		case INPUT:
			digital_pin.pullup = val == HIGH;
			break;
		case OUTPUT:
			digital_pin.value = val;
			break;
		case INPUT_PULLUP:
			throw std::invalid_argument("digitalWrite() for digital pin in INPUT_PULLUP mode is undefined behavior!");
			break;
		default:
			//throw std::invalid_argument(std::format("Pin's mode is set to illegal value 0x{:x}!", digital_pin.mode));
			throw std::invalid_argument("balls!");
			break;
	}
}
int digitalRead(uint8_t pin) {
	return digital_pins[pin].value;
}

int analogRead(uint8_t pin) {
	return analog_pins[pin].value;
}
// void analogReference(uint8_t mode) {} // Leave this unimplemented for now until we decide how to implement analog standards on TARGET_NATIVE
void analogWrite(uint8_t pin, int val) {
	AnalogPin& analog_pin = analog_pins[pin];

	if (analog_pin.mode != OUTPUT)
		//throw std::invalid_argument(std::format("analogWrite() for analog pin in any mode other than OUTPUT is undefined behavior!", digital_pin.mode));
		throw std::invalid_argument("idgaf");

	analog_pin.value = val;
}

String::String() : std::string() {}
String::String(const std::string& str) : std::string(str) {}
String::String(const char* s) : std::string(s) {}
String::String(const std::string& str, size_t pos, size_t len) : std::string(str, pos, len) {}

void String::trim() {
	size_t start = 0;
	for (;start < this->size() && std::isspace((*this)[start]); ++start);
	size_t end = this->size();
	for (;end > start && std::isspace((*this)[end - 1]); --end);

	this->erase(end);
	this->erase(0, start);
}


std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
void delay(unsigned long ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
void delayMicroseconds(unsigned long ms) {
	std::this_thread::sleep_for(std::chrono::microseconds(ms));
}

unsigned long millis() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}
unsigned long micros() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}

usb_serial_class::usb_serial_class() {}

void usb_serial_class::begin(long bitsPerSecond) {}
void usb_serial_class::setTimeout(long msTime) {}
size_t usb_serial_class::write(const char* msg, size_t len) {
	std::cout.write(msg, len);
	return len;
}
void usb_serial_class::flush() {
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}
size_t usb_serial_class::readBytes(char* buffer, size_t length) {
	size_t count = 0;
	for (char c; count < length && std::cin.get(c); buffer[count++] = c);
	buffer[count] = '\0';

	return count;
}
String usb_serial_class::readString(size_t length) {
	char* buffer = new char[length];
	this->readBytes(buffer, length);

	String str = String(buffer);
	delete buffer;

	return str;
}
size_t usb_serial_class::readBytesUntil(char terminator, char* buffer, size_t length) {
	size_t count = 0;
	for (char c; count < length && std::cin.get(c) && c != terminator; buffer[count++] = c);
	buffer[count] = '\0';

	return count;
}
String usb_serial_class::readStringUntil(char terminator, size_t length) {
	char* buffer = new char[length];
	this->readBytesUntil(terminator, buffer, length);

	String str = String(buffer);
	delete buffer;

	return str;
}

usb_serial_class Serial = usb_serial_class();


File::File() = default;
File::File(const char* path_str, char mode) {
	this->path = std::filesystem::path(BUILTIN_SDCARD_DIR);
	this->path /= path_str + (path_str[0] == '/' ? 1 : 0);
	this->name_str = this->path.filename().string();
	this->mode = mode;

	std::ios_base::openmode openMode = std::ios::binary;
	if (mode == FILE_READ)
			openMode |= std::ios::in;
	else if (mode == FILE_WRITE)
			openMode |= std::ios::in | std::ios::out;

	if (mode == FILE_WRITE) {
		std::ofstream createFile(this->path);
		createFile.close();
	}

	this->stream.open(this->path, openMode);

	if (std::filesystem::is_directory(this->path))
		this->dir_cursor = std::filesystem::directory_iterator(this->path);
}

File::operator bool() const {
	return !this->path.empty();
}
File File::openNextFile(uint8_t mode) {
	if (this->dir_cursor == this->dir_end)
		return File();

	const std::filesystem::directory_entry& entry = *this->dir_cursor;
	const std::string entry_str = entry.path().string();
	++this->dir_cursor;

	return File(entry_str.c_str());
}
const char* File::name() {
	return this->name_str.c_str();
}
void File::close() {
	this->stream.close();
}
bool File::available() {
	return this->stream.peek() != EOF;
}
void File::flush() {
	this->stream.flush();
}
size_t File::readBytes(char* buffer, size_t length) {
	size_t count = 0;
	for (char c; count < length && this->stream.get(c); buffer[count++] = c);
	buffer[count] = '\0';

	return count;
}
String File::readString(size_t length) {
	char* buffer = new char[length];
	buffer[this->readBytes(buffer, length)] = '\0';

	String str = String(buffer);
	delete buffer;

	return str;
}
size_t File::readBytesUntil(char terminator, char* buffer, size_t length, std::istream& stream) {
	size_t count = 0;
	for (char c; count < length && this->stream.get(c) && c != terminator; buffer[count++] = c);
	buffer[count] = '\0';

	return count;
}
String File::readStringUntil(char terminator, size_t length) {
	char* buffer = new char[length];
	buffer[this->readBytesUntil(terminator, buffer, length)] = '\0';

	String str = String(buffer);
	delete buffer;

	return str;
}


SDClass::SDClass() {}
bool SDClass::begin(uint8_t csPin) {
	if (!std::filesystem::is_directory(BUILTIN_SDCARD_DIR)) {
		try {
			std::filesystem::create_directory(BUILTIN_SDCARD_DIR);
		}
		catch (const std::exception& exc) {
			std::cout << "Failed to initialize BUILTIN_SDCARD_DIR and start the program becuase of the following error:" << exc.what() << std::endl;
			return false;
		}
	}

	return true;
}
File SDClass::open(const char* path_str, char mode) {
	return File(path_str, mode);
}
bool SDClass::exists(const char* file_path_str) {
	std::filesystem::path file_path = std::filesystem::path(BUILTIN_SDCARD_DIR);
	file_path /= file_path_str + (file_path_str[0] == '/' ? 1 : 0);
	return std::filesystem::exists(file_path);
}
bool SDClass::remove(const char* file_path_str) {
	std::filesystem::path file_path = std::filesystem::path(BUILTIN_SDCARD_DIR);
	file_path /= file_path_str + (file_path_str[0] == '/' ? 1 : 0);
	if (!std::filesystem::is_regular_file(file_path))
		return false;

	std::filesystem::remove(file_path);
	return true;
}

SDClass SD = SDClass();

void setup();
void loop();

int main() {
	setup();
	while (true)
		loop();
	return 0;
}

#endif