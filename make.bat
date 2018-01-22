@ECHO OFF

pushd %~dp0

REM Command file for Sphinx documentation

if "%SPHINXBUILD%" == "" (
	set SPHINXBUILD=sphinx-build
)
set SOURCEDIR=doc-source
set BUILDDIR=docs
set SPHINXPROJ=2018-PowerUp

REM files to be ignored by autodoc
set IGNORED_FILES=doc-source/

if "%1" == "" goto help

%SPHINXBUILD% >NUL 2>NUL
if errorlevel 9009 (
	echo.
	echo.The 'sphinx-build' command was not found. Make sure you have Sphinx
	echo.installed, then set the SPHINXBUILD environment variable to point
	echo.to the full path of the 'sphinx-build' executable. Alternatively you
	echo.may add the Sphinx directory to PATH.
	echo.
	echo.If you don't have Sphinx installed, grab it from
	echo.http://sphinx-doc.org/
	exit /b 1
)


if "%1" == "apidoc" goto apidoc
if "%1" == "html" goto apidoc

:apidoc
sphinx-apidoc -f -o doc-source/ . %IGNORED_FILES%
if "%1" == "apidoc" goto end

:html
%SPHINXBUILD% -b html %SOURCEDIR% %BUILDDIR% %SPHINXOPTS%
goto end

%SPHINXBUILD% -M %1 %SOURCEDIR% %BUILDDIR% %SPHINXOPTS%
goto end

:help
%SPHINXBUILD% -M help %SOURCEDIR% %BUILDDIR% %SPHINXOPTS%

:end
popd
