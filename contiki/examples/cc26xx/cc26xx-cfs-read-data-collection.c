#include "contiki.h"
#include <stdio.h>
#include "dev/leds.h"

#include <string.h>
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"

int v1;
int16_t v2;
int16_t v3;
int timestamp;
uint8_t delimiter;

#define TOTAL_RECORD_SIZE (int)(sizeof(v1) + sizeof(v2) + sizeof(v3) + sizeof(timestamp) + sizeof(delimiter))

#define FILENAME "data"

/*---------------------------------------------------------------------------*/
static int
dir_test(void)
{
  struct cfs_dir dir;
  struct cfs_dirent dirent;

  /* Coffee provides a root directory only. */
  if(cfs_opendir(&dir, "/") != 0) {
    printf("failed to open the root directory\n");
    return 0;
  }

  /* List all files and their file sizes. */
  printf("Available files\n");
  while(cfs_readdir(&dir, &dirent) == 0) {
    printf("%s (%lu bytes)\n", dirent.name, (unsigned long)dirent.size);
  }

  cfs_closedir(&dir);

  return 1;
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(read_external_flash, "Read external flash");
AUTOSTART_PROCESSES(&read_external_flash);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(read_external_flash, ev, data){
	PROCESS_BEGIN();

	printf("%d\n", TOTAL_RECORD_SIZE);

	int fd, r;

	if(dir_test() == 0) {
		printf("dir test failed\n");
	}

	printf("\n\n\n***START READING***\n\n\n");

	// read cfs file
	fd = cfs_open(FILENAME, CFS_READ);
	if(fd != -1) {
		if(cfs_seek(fd, 0, CFS_SEEK_SET) != 0) {
			printf("seek failed\n");
			cfs_close(fd);
		} else{
			leds_on(LEDS_ALL);
			for(;;) {
				r = cfs_read(fd, &v1, sizeof(v1));
				r += cfs_read(fd, &v2, sizeof(v2));
				r += cfs_read(fd, &v3, sizeof(v3));
				r += cfs_read(fd, &timestamp, sizeof(timestamp));
				r += cfs_read(fd, &delimiter, sizeof(delimiter));

				if(r == 0) {
					printf("break\n");
					break;
				} else if(r < TOTAL_RECORD_SIZE) {
					printf("failed to read %d bytes from %s, got %d\n", TOTAL_RECORD_SIZE, FILENAME, r);
					break;
				}

				printf("%d,%d,%d,%d,%d\n", delimiter, timestamp, v1, v2, v3);
			}
			cfs_close(fd);
			printf("\n\n\n***END READING***\n\n\n");

			leds_off(LEDS_ALL);
		}
	} else{
		printf("failed to open %s\n", FILENAME);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/