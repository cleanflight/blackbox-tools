/*
 * This tool is for designing the intervals to capture P-frames in for
 * given numerator/denominators chosen by the user.
 */

#include <stdio.h>

#define IFRAME_INTERVAL 64

int gcd(int num, int denom) {
	if (denom == 0)
		return num;
	return gcd(denom, num % denom);
}

void printPattern(int num, int denom, int shift) {
	int filled = 0;
	int div = gcd(num, denom);

	printf("%2d/%2d shift %d ", num, denom, shift);

	num /= div;
	denom /= div;

	for (int i = 0; i <= IFRAME_INTERVAL * 2; i++) {
		int j = i % IFRAME_INTERVAL;

		if (j == 0) {
			printf("I");
			filled++;
		} else {
			if (denom == 1) {
				printf("P");
				filled++;
			} else {
				if ((j + shift) % denom < num) {
					printf("P");
					filled++;
				} else {
					printf(".");
				}
			}
		}
	}
	printf(" %4.2f\n", ((double)filled - 1) / (IFRAME_INTERVAL * 2));
}

void printPattern2(int num, int denom) {
	int shift;
	int filled = 0;
	int div = gcd(num, denom);

	num /= div;
	denom /= div;

	shift = num - 1;

	printf("%2d/%2d shift %d ", num * div, denom * div, shift);

	for (int i = 0; i <= IFRAME_INTERVAL * 2; i++) {
		int j = i % IFRAME_INTERVAL;

		if (j == 0) {
			filled++;
			printf("I");
		} else {
			if (denom == 1) {
				printf("P");
				filled++;
			} else {
				if ((j + shift) % denom < num) {
					printf("P");
					filled++;
				} else {
					printf(".");
				}
			}
		}
	}

	printf(" %4.2f\n", ((double)filled - 1) / (IFRAME_INTERVAL * 2));
}

int main(void) {
	//Try out some different shifts for each pattern to spot the best ones:
	printPattern(1, 1, 0);
	printf("\n");
	printPattern(1, 2, 0);
	printPattern(1, 2, 1);
	printf("\n");
	printPattern(1, 3, 0);
	printPattern(1, 3, 1);
	printPattern(1, 3, 2);
	printf("\n");
	printPattern(2, 3, 0);
	printPattern(2, 3, 1);
	printPattern(2, 3, 2);
	printf("\n");
	printPattern(1, 4, 0);
	printPattern(1, 4, 1);
	printPattern(1, 4, 2);
	printPattern(1, 4, 3);
	printf("\n");
	printPattern(2, 4, 0);
	printPattern(2, 4, 1);
	printPattern(2, 4, 2);
	printPattern(2, 4, 3);
	printf("\n");
	printPattern(3, 4, 0);
	printPattern(3, 4, 1);
	printPattern(3, 4, 2);
	printPattern(3, 4, 3);
	printf("\n");
	printPattern(1, 8, 0);
	printPattern(1, 8, 1);
	printPattern(1, 8, 2);
	printPattern(1, 8, 3);
	printf("\n");
	printPattern(2, 8, 0);
	printPattern(2, 8, 1);
	printPattern(2, 8, 2);
	printPattern(2, 8, 3);
	printf("\n");
	printf("\n");

	//Try an algorithm that picks the best shift itself:
	for (int denom = 1; denom <= 8; denom++)
		for (int num = 1; num <= denom; num++)
			printPattern2(num, denom);
	printf("\n");

	//This should be the ugliest case
    printPattern2(IFRAME_INTERVAL / 2 - 1, IFRAME_INTERVAL);

	return 0;
}
