#ifndef __CCI_HW_ID_H
#define __CCI_HW_ID_H

/* Implement CCI HW ID/PROJECT ID */

#define CEI_HWID_STRING_LEN 10

#define NUM_OF_HWID_GPIO 3
#define NUM_OF_PROJID_GPIO 4

/**
  * HWID1 | HWID 2 | HWID 3 
  * GPIO90   GPIO89   GPIO58
  *
  * 000-001-010-011-100-101-110-111
  * EVT-DVT1_1-DVT1_2-DVT2-DVT3-TP-PVT-MP
  * 
**/

enum cei_hw_type {
	/* SM11 */
	CEI_HW_EVT       = 0,       /* PDP */
	CEI_HW_DVT1_1 = 1,       /* DP1  */
	CEI_HW_DVT1_2 = 2,       /* DP2  */
	CEI_HW_DVT2    = 3,       /* SP    */
	CEI_HW_DVT3    = 4,       /* AP    */	
	CEI_HW_TP      = 5,         /* TP   */
	CEI_HW_DVT4    = 6,       /* AP2 */
	CEI_HW_PVT     = 7,         /* PQ   */
	/* SM21 */
	CEI_SM21_HW_EVT       = 0,       /* PDP */
	CEI_SM21_HW_DVT1_1 = 1,       /* SP1  */
	CEI_SM21_HW_DVT1_2 = 2,       /* SP2  */
	CEI_SM21_HW_DVT2    = 3,       /* AP    */
	CEI_SM21_HW_TP      = 4,         /* TP   */
	CEI_SM21_HW_PVT     = 5,        /* PQ   */
	CEI_SM21_HW_MP      = 6         /* MP   */	
};

/**
  * HWID4   | HWID8   | HWID9 | HWID10 |
  * GPIO59  | GPIO56  | GPIO57 | GPIO80 |
  * <-CAT->|<-SIM->| <-      SKU       -> |
  *
  * 0000-0001-0010-1000-1100-1101-1010
  *
**/

enum cei_project_type {
	CEI_PROJECT_BY57 = 0,                                   /* CAT6 GINA SS */
	CEI_PROJECT_BY86 = CEI_PROJECT_BY57,  /* CAT6 GINA SS */
	CEI_PROJECT_BY61 = CEI_PROJECT_BY57,  /* CAT6 GINA SS */
	CEI_PROJECT_BY58 = 1,                                   /* CAT6 GINA APAC SS */
	CEI_PROJECT_BY59 = 2,                                   /* CAT6 REX SS */
	CEI_PROJECT_BY88 = CEI_PROJECT_BY59, /* CAT6 REX SS */
	CEI_PROJECT_BY65 = CEI_PROJECT_BY59, /* CAT6 REX SS */
	CEI_PROJECT_BY78 = 3,                                   /* CAT4 REX SS */      /* Removed */
	CEI_PROJECT_BY87 = 5,                                   /* CAT6 GINA APAC DS */
	CEI_PROJECT_BY63 = CEI_PROJECT_BY87,          /* CAT6 GINA APAC DS */
	CEI_PROJECT_BY66 = 8,                                   /* CAT4 GINA SS*/
	CEI_PROJECT_BY77 = 9,                                   /* CAT4 APAC SS */   /* Removed */
	CEI_PROJECT_BY69 = 10,                                 /* CAT4 REX SS */      /* Removed */
	CEI_PROJECT_BY78_EVT = CEI_PROJECT_BY69,  /* CAT4 REX SS SM21 EVT */
	CEI_PROJECT_BY67 = 12,                                 /* CAT4 GINA DS */
	CEI_PROJECT_BY76 = CEI_PROJECT_BY67,  /* CAT4 GINA DS */
	CEI_PROJECT_BY62 = CEI_PROJECT_BY67,  /* CAT4 GINA DS */
	CEI_PROJECT_BY68 = 13,                                  /* CAT4 GINA APAC DS */
	CEI_PROJECT_BY64 = CEI_PROJECT_BY68           /* CAT4 GINA APAC DS */
};

enum cei_sim_type {
	CEI_SIM_SS = 0,
	CEI_SIM_DS = 1
};

enum cei_rf_type {
	CEI_RF_GINA = 0,
	CEI_RF_GINA_APAC = 1,
	CEI_RF_REX = 2
};

enum cei_lte_type {
	CEI_LTE_CAT6 = 0,
	CEI_LTE_CAT4 = 1
};

enum cei_customer_project_type {
	CUSTOMER_PROJECT_SM11 = 0,
	CUSTOMER_PROJECT_SM21 = 1,
	CUSTOMER_PROJECT_SM10N = 2,
	CUSTOMER_PROJECT_SM11L = 3
};

/*
 * API to get CEI HWID information:
 *
 * get_cei_hw_id()-      return enum cei_hw_type
 * get_cei_project_id()- return enum cei_project_type
 * get_cei_customer_project_id()- return cei_customer_project_type
 * get_cei_lte_id() - return cei_lte_type
 * get_cei_sim_id() - return cei_sim_type
 * get_cei_rf_id() - return cei_rf_type
 *
 * Please use *enum variable* in your function to get the return cei_XXX_type
 */
 
extern enum cei_hw_type get_cei_hw_id(void); 
extern enum cei_project_type get_cei_project_id(void);
extern enum cei_customer_project_type get_cei_customer_project_id(void);
extern enum cei_lte_type get_cei_lte_id(void);
extern enum cei_sim_type get_cei_sim_id(void);
extern enum cei_rf_type get_cei_rf_id(void);

#endif /* __CCI_HW_ID_H */
