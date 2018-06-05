::@echo off 

@for /d /r %%c in (settings) do @if exist %%c ( rd /s /q "%%c" & echo     É¾³ýÄ¿Â¼%%c) 


@for /d /r %%c in (Debug\List) do @if exist %%c ( rd /s /q "%%c" & echo     É¾³ýÄ¿Â¼%%c) 
@for /d /r %%c in (Debug\Obj) do @if exist %%c ( rd /s /q "%%c" & echo     É¾³ýÄ¿Â¼%%c) 

@for /d /r %%c in (Release\Obj) do @if exist %%c ( rd /s /q "%%c" & echo     É¾³ýÄ¿Â¼%%c) 


@for /r  %%c in (*.dep ) do del "%%c"
@for /r  %%c in (*.sfr ) do del "%%c"
@for /r  %%c in (*.tmp ) do del "%%c"


pause