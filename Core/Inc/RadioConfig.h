#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

/* Keep identical in FlightController and RemoteController.
 * Both boards must use the same bandwidth and be reflashed together.
 * 250000u speeds up packets; 125000u provides better receiver sensitivity.
 * This setting applies to control packets as well as log downloads.
 * May also be overridden with -DRADIO_BANDWIDTH_HZ=125000u.
 */
#ifndef RADIO_BANDWIDTH_HZ
#define RADIO_BANDWIDTH_HZ 250000u
#endif

#if (RADIO_BANDWIDTH_HZ != 125000u) && (RADIO_BANDWIDTH_HZ != 250000u)
#error "RADIO_BANDWIDTH_HZ must be 125000 or 250000; match both boards"
#endif

#endif /* RADIO_CONFIG_H */
