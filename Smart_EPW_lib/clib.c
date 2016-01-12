
#include <stddef.h>
#include <stdint.h>
#include <limits.h>

#define ALIGN (sizeof(size_t))
#define ONES ((size_t)-1/UCHAR_MAX)                                                                      
#define HIGHS (ONES * (UCHAR_MAX/2+1))
#define HASZERO(x) ((x)-ONES & ~(x) & HIGHS)

#define SS (sizeof(size_t))
void *memset(void *dest, int c, size_t n)
{
		unsigned char *s = dest;
		c = (unsigned char)c;
		for (; ((uintptr_t)s & ALIGN) && n; n--) *s++ = c;
		if (n) {
				size_t *w, k = ONES * c;
				for (w = (void *)s; n>=SS; n-=SS, w++) *w = k;
				for (s = (void *)w; n; n--, s++) *s = c;
		}
		return dest;
}

void *memcpy(void *dest, const void *src, size_t n)
{
		void *ret = dest;

		//Cut rear
		uint8_t *dst8 = dest;
		const uint8_t *src8 = src;
		switch (n % 4) {
				case 3 : *dst8++ = *src8++;
				case 2 : *dst8++ = *src8++;
				case 1 : *dst8++ = *src8++;
				case 0 : ;
		}

		//stm32 data bus width
		uint32_t *dst32 = (void *)dst8;
		const uint32_t *src32 = (void *)src8;
		n = n / 4;
		while (n--) {
				*dst32++ = *src32++;
		}

		return ret;
}

char *strchr(const char *s, int c)
{
		for (; *s && *s != c; s++);
		return (*s == c) ? (char *)s : NULL;
}

char *strcpy(char *dest, const char *src)
{
		const unsigned char *s = src;
		unsigned char *d = dest;
		while ((*d++ = *s++));
		return dest;
}

char *strncpy(char *dest, const char *src, size_t n)
{
		const unsigned char *s = src;
		unsigned char *d = dest;
		while (n-- && (*d++ = *s++));
		return dest;
}
int strlen(char *str )
{
		int i = 0 ;
		while(str[i]!='\0'){
				i++;
		}
		return i ;
}
int strcmp(const char* s1, const char* s2)
{
		while(*s1 && (*s1==*s2))
				s1++,s2++;
		return *(const unsigned char*)s1-*(const unsigned char*)s2;
}
int strncmp(const char *s1 , const char *s2 , size_t n)
{
		int i ;
		for(i = 0; i < n; i++) {
				if (s1[i] != s2[i]) {
						return s1[i] - s2[i];
				}
		}
		return 0;
} 
char *strcat(char *dest, char *src)
{
		for (;*dest;dest++); 
		while(*dest++ = *src++);
		return dest ;
}  


int round(float number)
{
		return   number > 0 ?  (number + 0.5f) : (number - 0.5f);
}


