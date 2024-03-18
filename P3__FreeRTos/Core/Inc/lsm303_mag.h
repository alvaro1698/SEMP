#ifndef _LSM303_MAG_H_
#define _LSM303_MAG_H_

#include "../../Components/lsm303agr/lsm303agr.h"

void LSM303AGR_MagInit(void);
void LSM303AGR_MagReadXYZ(int16_t *pData);

#endif /* _LSM303_MAG_H_ */
