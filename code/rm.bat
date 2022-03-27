@echo off
rem setlocal enabledelayedexpansion

for %%x in (%*) do (
   del /Q %%~x
)