    ;TTL tarif
    ;AREA program, CODE ,READONLY
	;ENTRY
	;LDR r0,=0xC0000000 ;1100
	;LDR r1,=0x80000000 ;1000
    ;ADDS r2,r0,r1      ;0100
    ;END
	MOV r2,#0
	;TTL	Ders1
	;AREA	Program, CODE, READONLY
	;ENTRY			
	;LDR r0,=2_101 ;saga kaydirma
	;LSL r1,r0,#7 ; 7 tane kaydirma
	;MOV r2,#0 ; 
	;END
	
	