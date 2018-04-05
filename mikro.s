    ;TTL tarif
    ;AREA program, CODE ,READONLY
	;ENTRY
	;LDR r0,=0xC0000000 ;1100
	;LDR r1,=0x80000000 ;1000
    ;ADDS r2,r0,r1      ;0100
    ;END
	TTL	Ders1
	AREA	Program, CODE, READONLY
	ENTRY			
	LDR r0,=2_101
	LSL r1,r0,#7 
	MOV r2,#0
	END
	