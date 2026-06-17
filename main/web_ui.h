/**
 * Web Diagnostics and Configuration UI
 *
 * Hosts a small local HTTP server with:
 * - status dashboard
 * - live telemetry JSON
 * - EXLAP config view/update
 */

#ifndef WEB_UI_H
#define WEB_UI_H

#include <stdbool.h>

bool web_ui_init(void);
void web_ui_stop(void);

#endif /* WEB_UI_H */
