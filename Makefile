All:	ctrlrFTR15 btctrl mnul_btctrl

ctrlrFTR15:	ctrlrFTR15.c
#	gcc -o ctrlrFTR15 ctrlrFTR15.c -lbluetooth -lpthread
	gcc -o ctrlrFTR15 ctrlrFTR15.c -lpthread -Wall

btctrl:	btctrl.c
#	gcc -o btctrl btctrl.c -lbluetooth -lpthread
	gcc -o btctrl btctrl.c -lpthread -Wall

mnul_btctrl:	mnul_btctrl.c
	gcc -o mnul_btctrl mnul_btctrl.c -lpthread -Wall


