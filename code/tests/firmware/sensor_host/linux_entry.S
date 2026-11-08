# Only used by the Windows Clang + WSL libc fallback (no compiler installation).
.text
.global _start
_start:
  xor %ebp, %ebp
  andq $-16, %rsp
  call main
  movl %eax, %edi
  call exit
.section .note.GNU-stack,"",@progbits
