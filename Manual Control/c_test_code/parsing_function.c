#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUF_SIZE 128

// Function to parse the input string
void parse_input_string(char *input) {
    printf("Parsing input string: %s\n", input);

    // Split and just get the first word
    char *token = strtok(input, " "); 

    printf("Parsing for state we have: %s\n", input);

    // Simulate switch-case behavior with strcmp for string comparison
    if (strcmp(input, "MOV") == 0) {
        printf("The bot is moving.\n");
    } 
    else if (strcmp(input, "STARTUP") == 0) {
        printf("The bot is starting up.\n");
    } 
    else {
        printf("Unknown command received: %s\n", input);
    } 
}

int main() {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    if (!data) {
        fprintf(stderr, "Failed to allocate memory for data buffer\n");
        return 1;
    }

    // Simulate receiving "MOV L"
    // strncpy((char *)data, "MOV L", BUF_SIZE - 1);
    strncpy((char *)data, "STARTUP L", BUF_SIZE - 1);

    data[BUF_SIZE - 1] = '\0'; // Ensure null-termination

    // parse the string
    parse_input_string((char *)data);

    free(data); // Clean up
    return 0;
}