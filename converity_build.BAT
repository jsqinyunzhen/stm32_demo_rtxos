rem @echo off

rem �趨��������������
rem set path=%path%;C:\Program Files\TortoiseSVN\bin\svn.exe
set LANG=zh_CN.UTF-8
set coverity_dir=ci_coverity_dir
rem %JOB_NAME%_%BUILD_ID%
rem ����Coverity��log���Ŀ¼
echo %WORKSPACE%
mkdir %coverity_dir%
rem ִ�б���
cov-build.exe --dir "%WORKSPACE%\%coverity_dir%" invoke_build.BAT
rem �ж��Ƿ�ɹ�
if %errorlevel% NEQ 0 (
echo ">>>>>>>>>>>>>>build faild<<<<<<<<<<<<<<<<<<<<"
for /f "delims=" %%i in ('dir "ci_*" /ad/b') do rd "%%i" /s/q
exit /b 1
)
rem ��ȡSVN��Ϣ����ƥ��
rem "D:\IDE\cov-analysis-win64-8.7.0\bin\cov-import-scm" --dir %WORKSPACE%\%coverity_dir% --scm svn
rem ���Coverity�Ѽ��ı��������Ϣ
rem "C:\Program Files (x86)\7-Zip\7z" a %coverity_dir%.zip %coverity_dir%