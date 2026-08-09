#include <iostream>
#include <cstdlib>
#include <ctime>

void status(){
    const char* statuses[] = {
        "ROBOT STATUS: OK",
        "ROBOT STATUS: Low Battery",
        "ROBOT STATUS: Obstacle Detected",
        "ROBOT STATUS: Overheating"
    };
    
    int num = rand() % 4;
    std::cout << statuses[num] << std::endl;
}

int main(){
    srand(time(0));
    status();

    return 0;
}
