; Configure sensor #5
	START
	SEND	5,W	; Device address, we will be writing
	SEND	15	; Address within sensor 5
	SEND	16	; MEM5[15] = 16
	SEND	32	; MEM5[16] = 32
	SEND	128	; MEM5[17] = 128
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
	CHANNEL 5
	START
	SEND	5,WR	; Device address, we will start out writing
	SEND	22	; Pointer byte
	START		; Repeated start
	SEND	5,RD	; Device address, now we are reading
	RXK		; Read four bytes
	RXK
	RXK
	RXN		; Do not ACK the last byte
	STOP
	ABORT
; Read from sensor 22, address 22
	START
	SEND	22,WR
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
