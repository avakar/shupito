.section .init1

fill_stack_canary:
	ldi r30, lo8(_end)
	ldi r31, hi8(_end)
	ldi r24, 0xc5
	ldi r25, hi8(__stack)
	rjmp .cmp

.loop:
	st Z+, r24

.cmp:
	cpi r30, lo8(__stack)
	cpc r31, r25
	brlo .loop
	breq .loop
