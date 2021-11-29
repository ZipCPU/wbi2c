; Configure sensor #5
	START
	SEND	5,W
	SEND	15
	SEND	16
	SEND	32
	SEND	128
	STOP
; Configure sensor #22
	START
	SEND	22,W
	SEND	96
	SEND	48
	STOP
	TARGET
; Wait for sync
	WAIT
	ABORT
; Read from sensor 5, address 22
	START
	SEND	5,RD
	SEND	22
	START
	SEND	5,RD
	RXK
	RXK
	RXK
	RXN
	STOP
	ABORT
; Read from sensor 22, address 22
	START
	SEND	22,RD
	SEND	22
	START
	SEND	22,RD
	RXK
	RXK
	RXK
	RXLN
	STOP
	JUMP
	HALT
