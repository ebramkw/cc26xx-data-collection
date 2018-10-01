#include "contiki.h"
#include "dev/serial-line.h"

PROCESS(main_process, "main process");
AUTOSTART_PROCESSES(&main_process);

PROCESS_THREAD(main_process, ev, data)
{
	printf("Hello\n");
    PROCESS_BEGIN();
    for(;;) {
        PROCESS_WAIT_EVENT();

        if (ev == serial_line_event_message && data != NULL) {
           printf("got input string: '%s'\n", (const char *) data);
        }
    }
    PROCESS_END();
}