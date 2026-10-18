#pragma once
// Minimal Arduino shim for compiling the REAL src/Vision/Vision.cpp on the
// host (test/vision_link_test.cpp): a std::string-backed String with the
// methods Vision.cpp uses, a fake Serial1 (bytes in from the "Orin", lines
// out from the Mega) and a settable millis().
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>

namespace visionstub {
extern unsigned long nowMs;
}

inline unsigned long millis() { return visionstub::nowMs; }

template <typename T>
T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

class String {
public:
    String() {}
    String(const char* s) : s_(s) {}
    void reserve(unsigned int n) { s_.reserve(n); }
    unsigned int length() const { return static_cast<unsigned int>(s_.size()); }
    char charAt(unsigned int i) const { return i < s_.size() ? s_[i] : '\0'; }
    const char* c_str() const { return s_.c_str(); }
    bool startsWith(const char* p) const { return s_.rfind(p, 0) == 0; }
    bool startsWith(const String& p) const { return s_.rfind(p.s_, 0) == 0; }
    int indexOf(char c, unsigned int from = 0) const {
        const size_t i = s_.find(c, from);
        return i == std::string::npos ? -1 : static_cast<int>(i);
    }
    String substring(unsigned int from) const { return from >= s_.size() ? String() : String(s_.substr(from).c_str()); }
    String substring(unsigned int from, unsigned int to) const {
        if (from >= s_.size() || to <= from) return String();
        return String(s_.substr(from, to - from).c_str());
    }
    void trim() {
        const size_t a = s_.find_first_not_of(" \t\r\n");
        const size_t b = s_.find_last_not_of(" \t\r\n");
        s_ = a == std::string::npos ? std::string() : s_.substr(a, b - a + 1);
    }
    String& operator+=(char c) { s_ += c; return *this; }
    bool operator==(const char* o) const { return s_ == o; }

private:
    std::string s_;
};

class FakeSerial1 {
public:
    std::string input;   // bytes the Orin sent
    std::string output;  // what the Mega wrote
    void begin(unsigned long) {}
    int available() { return static_cast<int>(input.size()); }
    int read() {
        if (input.empty()) return -1;
        const char c = input[0];
        input.erase(0, 1);
        return static_cast<unsigned char>(c);
    }
    void print(const char* s) { output += s; }
    void print(char c) { output += c; }
    void print(int v) { output += std::to_string(v); }
    void print(unsigned char v) { output += std::to_string(static_cast<unsigned>(v)); }
};
extern FakeSerial1 Serial1;
