.global hiv_compute_per

; #define RES0 r24
#define RES1 r21
#define RES2 r22
#define RES3 r23
#define RES4 r24
#define RES5 r25

#define TMP0 r26
#define TMP1 r27
#define TMP2 r30
#define TMP3 r31

; // setpoint_recip <= 0xfffff
; // old_per <= 0xfff
; // about 130 cycles
; uint16_t hiv_compute_per(uint16_t old_per, uint8_t cur_voltage, uint32_t setpoint_recip);
hiv_compute_per:
	; old_per        : r25:r24
	; cur_voltage    : r22
	; setpoint_recip : r20:r19:r18
	; result         : r25:r24

	push r17
	clr r17

	mul r22, r22
	movw r22, r0   ; r23:r22 = cur2 = cur_voltage * cur_voltage

	mul r22, r24
	movw TMP0, r0   ; r15:r14 = perL * cur2L

	mul r23, r25
	movw TMP2, r0

	mul r23, r24
	add TMP1, r0
	adc TMP2, r1
	adc TMP3, r17

	mul r22, r25
	add TMP1, r0
	adc TMP2, r1
	adc TMP3, r17

	;           TMP = old_per * cur2 = tmp
	;   r20:r19:r18 = setpoint_recip
	; TMP is only 28 bits wide, multiply by 20-bit setpoint_recip to get 48 bits of result
	; we discard the lower bytes as soon as they're no longer necessary; we never need RES0
	mul TMP0, r18
	mov RES1, r1

	mul TMP1, r19
	movw RES2, r0

	mul TMP2, r20
	movw RES4, r0

	mul TMP1, r18
	add RES1, r0
	adc RES2, r1
	adc RES3, r17
	adc RES4, r17
	adc RES5, r17

	mul TMP0, r19
	add RES1, r0
	adc RES2, r1
	adc RES3, r17
	adc RES4, r17
	adc RES5, r17

	mul TMP2, r18
	add RES2, r0
	adc RES3, r1
	adc RES4, r17
	adc RES5, r17

	mul TMP3, r18
	add RES3, r0
	adc RES4, r1
	adc RES5, r17

	mul TMP2, r19
	add RES3, r0
	adc RES4, r1
	adc RES5, r17

	mul TMP3, r19
	add RES4, r0
	adc RES5, r1

	mul TMP0, r20
	add RES2, r0
	adc RES3, r1
	adc RES4, r17
	adc RES5, r17

	mul TMP1, r20
	add RES3, r0
	adc RES4, r1
	adc RES5, r17

	mul TMP3, r20
	add RES5, r0

	pop r17
	clr r1
	ret
