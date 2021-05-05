#include <iostream>
#include <string>

std::string getPathName(const char *fifopath)
{
    FILE *f = fopen(fifopath, "rb");
    char b[FILENAME_MAX];
    fread(b, FILENAME_MAX, 1, f);
    fclose(f);
    return std::string(b);
}

int main()
{
    while (1)
        std::cout << getPathName("fifo") << std::endl;
    return 0;
}