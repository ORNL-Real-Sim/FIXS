/*
******************************************************************************
**  CarMaker - Version $CarMakerVersion$
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#ifndef _RMA_TOPOLOGY_H_
#define _RMA_TOPOLOGY_H_

#if defined(MODEL)
# define RMA_QUOTE1(name)	#name
# define RMA_QUOTE(name)	RMA_QUOTE1(name)

const char RMA_ModelName[] = RMA_QUOTE(MODEL);
#else
extern const char RMA_ModelName[];
#endif


typedef struct tRMAt_Node {
    const char   *Name;
    unsigned int AppID;
} tRMAt_Node;

const tRMAt_Node RMAt_Table[] = {
    { "CM13_2024a_MachE_for_UA_HIL", 1 },
    { NULL, 0 }
};


#define RMA_MasterApp	"CM13_2024a_MachE_for_UA_HIL"

#define RMA_BufId_Offset	2048
#define RMA_SendBufID(n, i)	(RMA_BufId_Offset + 4*(n)+2*(i)+1)
#define RMA_RecvBufID(n, i)	(RMA_BufId_Offset + 4*(n)+2*(i))
#define RMAc_SendBufID(n, i)	(RMA_BufId_Offset + 4*(n)+2*(i))
#define RMAc_RecvBufID(n, i)	(RMA_BufId_Offset + 4*(n)+2*(i)+1)

#endif	/* _RMA_TOPOLOGY_H_ */
