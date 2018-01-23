@echo off
:: Author: stojand

:: Define path of your KST \bin folder
set PATH=%PATH%;D:\Programs\Kst-2.0.x-2017.08.19-00.34-win32\bin

pushd D:\stojand\Projekti\WaPu_FOC\c programming\mcsim20171003_jc_ds\mcsim20171003_jc

start kst2 kst_plot_inverter_slow.kst
