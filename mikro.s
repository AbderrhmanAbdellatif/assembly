    ;TTL tarif
    ;AREA program, CODE ,READONLY
	;ENTRY
	;LDR r0,=0xC0000000 ;1100
	;LDR r1,=0x80000000 ;1000
    ;ADDS r2,r0,r1      ;0100
    ;END
	;TTL	Ders1
	;AREA	Program, CODE, READONLY
	;ENTRY			
	;LDR r0,=2_101 ;saga kaydirma
	;LSL r1,r0,#7 ; 7 tane kaydirma
	;MOV r2,#0 ; 
	;END
    ;TTL	Ders1
	;AREA	Program, CODE, READONLY
	;ENTRY			
	;LDR r0,=0x80000002
	;LSLS r1,r0,#1
	;MOV r2,#0
    ;stop B stop
	;END
   ;TTL bitopartetor
   ;AREA Program,CODE,READONLY
   ;ENTRY   
   ;LDR r0,=0xD0000000
   ;LDR r1,=0x50000000
   ;ADDS r3,r0,r1
   ;END
   ;LDR r0,=0X0098FAB0
	;MOV r1,#7
	;ASRS r3,r0,r1
	;END
		;;0 x 0 0 0 1 3 1 F 5
	    ;TTL	Ders1
	;AREA	Program, CODE, READONLY
	;ENTRY
	;LDR r0,=0XC054798A
	;MOV r1,#13
	;RORS r2,r0,r1
	;END
		;;0 x 0 0 0 1 3 1 F 5

	;Soru 1: N! Hesabi yapan assembly program:
    ;TTL	Ders1
	;AREA	Program, CODE, READONLY
	;ENTRY
	;MOV r1,#4		;n 
	;LDR R0,=1		; r0 = n!
	;MOV r2,r1		;temp
;bas
	;CMP r2,#1
	;BLO son			;{
	;MUL r0,r2,r0
	;SUB r2,r2,#1
	;B bas			;}	
;son
	;END
   ;TTL ders1
   ;AREA	Program, CODE, READONLY
   ;ENTRY
   ;MOV r0,#10 ; int n =toplayacagimiz
   ;LDR R1,=0  ;int toplam=0
   ;MOV r2,#1   ;int counter
   ;CMP r2,r0  ;if (counter <= N)
;niyatwhile   
   ;BGT sartyanlis   ;sart sagliyor
   ;ADD r1,R1,r2     ; sum  +=conter
   ;ADD r2,R2,#1     ;counter=counter+1
   ;B niyatwhile
;sartyanlis
   ;END
   
	
	
