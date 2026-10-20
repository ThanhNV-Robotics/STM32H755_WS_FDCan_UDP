/**
 * tool_tunnel.h - nucleotool UDP tunnel on CM7 (port 9998), forwards CANdle commands to CM4
 *
 * Separate from the WBC server (udp_server.c, port 9999): own pcb, own state.
 */
#ifndef TOOL_TUNNEL_H
#define TOOL_TUNNEL_H

#include <stdbool.h>
#include <stdint.h>

bool ToolTunnel_Init(void);
/* Main loop: send the UDP reply once CM4 has answered. Never blocks. */
void ToolTunnel_Poll(void);

#endif /* TOOL_TUNNEL_H */
