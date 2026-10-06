// Runtime adapter for the chip-independent observer in algorithm/.
#ifndef FOC_SMO_OBSERVER_ADAPTER_H
#define FOC_SMO_OBSERVER_ADAPTER_H

#include "motor_runtime.h"
#include "algorithm/sliding_mode_observer.h"

/* Runtime name kept for callers; state layout belongs to the pure algorithm. */
typedef SMO_ObserverState_t SMO_Observer_t;

void SMO_Observer_Init(SMO_Observer_t *smo);
void SMO_Observer_Update(SMO_Observer_t *smo, MOTOR_DATA *motor);

#endif /* FOC_SMO_OBSERVER_ADAPTER_H */
