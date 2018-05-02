 AREA PROGRAM, CODE,READONLY
    ARM
   ENTRY
    ;MOV r0,#1  ;int i=1
	;LDR r1,=0x00001111 ;baslangic adres 
;devam 
    ;CMP r0,#15 ; i<=15
	;BGE cik      
	;STRH r0,[r1],#2 ; (short)arrey[i]=i+1
	;ADD r0,r0,#1 ;i+1
	;B devam ; while (ture)
;cik
    ;MOV r0,#1
	;MOV r4,#0
    ;LDR r1,=0x00000001
;davem    
	;CMP r0,#30 ; ldr saddece ilerime ama yazma  olmuyor
    ;BGE cik
   ;; STR r0,[r1],#4
    ;LDR r3,[r1],#4;buraada 4 dafe ileriyor ama yazmiyor ve mantigi soyle r1 ileriyor   arryedeki elmenleri okuyup bana geri veriyor 
   ;;ADD r0,r0,r4 
    ;ADD r4,r4,r3
    ;ADD r0,r0,#1	
    ;B davem
;;cik   
    ;LDR r0,=0x00001000  ;baseadress
	;MOV r1,#1
	;MOV r3,#2
;loop	
	;CMP r1,#15
	;BGE cik
	;STR r1,[r0],#4
	;MUL r1,r3,r1
	;ADD r1,r1,#1
	;B loop
;cik	
	;LDR r1, =0x00000000	; base adress of byte array
	;MOV r2,#1				;counter
;loop
	;CMP r2,#15
	;BGT	cik
	;STRB r2,[r1],#1			;b[i] = (i+1); 
	;ADD r2,R2,#1			;i++
	;B loop
;cik

   END 
	