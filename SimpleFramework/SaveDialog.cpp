#include <stdlib.h>
#include <string.h>

#include <windows.h>

#include "SaveDialog.h"

char* save_dialog(const char* path) {
	char* buf = (char*)malloc(1024);
	strcpy(buf, path);
	buf[strlen(path)] = '\0';

	OPENFILENAMEA f = { 0 };

	f.lStructSize = sizeof(f);
	f.lpstrFile = buf;
	f.nMaxFile = 1024;
	f.Flags = OFN_EXPLORER | OFN_HIDEREADONLY;

	if (GetSaveFileNameA(&f)) {
		return buf;
	}

	free(buf);
	return nullptr;
}