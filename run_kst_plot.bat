@echo off
:: Author: stojand

:: Define path of your KST \bin folder
:: set PATH=%PATH%;D:\Programs\Kst-2.0.x-2017.08.19-00.34-win32\bin
set PATH=%PATH%;C:\Program Files (x86)\Kst\bin

pushd %~dp0

start kst2 kst_plot_inverter_slow.kst
