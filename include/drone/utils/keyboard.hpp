#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>

namespace drone
{

/// Check if a key has been pressed (non-blocking).
/// Returns false if stdin is not a terminal.
inline bool kbhit()
{
  struct termios oldt{};
  if (tcgetattr(STDIN_FILENO, &oldt) != 0) return false;

  struct termios newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

  int ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, old_flags);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return true;
  }
  return false;
}

/// Read a single character without echo.
/// Returns '\0' if stdin is not a terminal.
inline char getch()
{
  struct termios oldt{};
  if (tcgetattr(STDIN_FILENO, &oldt) != 0) return '\0';

  struct termios newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char ch = static_cast<char>(getchar());

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

}  // namespace drone
