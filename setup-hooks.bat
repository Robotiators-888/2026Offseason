@rem This can be invoked using .\setup-hooks or .\setup-hooks.bat because of how windows handles filenames for scripts
@rem This recursivley copies files from .hooks to .git\hooks
@rem /s and /e copies files and subfolders (there aren't any subfolders) recursivley and /Y gets rid of the confirmation prompt
xcopy .\.hooks .\.git\hooks\ /s /e /Y