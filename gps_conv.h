static double nmea_latlon (char *s)
{
	int i, dot, begin;
	double result, factor;

	/* set 'begin' to start of degrees */
	for (begin=0;s[begin] < '0' || s[begin] > '9';begin++)
		;

	/* set 'dot' to where the dot is placed */
	for (dot=begin+1; s[dot] != '.'; dot++)
		;

	/* retrieve minutes */
	result = (s[dot-2]-'0') * 10 + (s[dot-1]-'0');

	/* retrieve decimal minutes */
	for (i=dot+1, factor=10; i<=(dot+7);i++, factor *= 10)
		if (s[i]>='0' && s[i]<='9')
			result += (s[i]-'0')/factor;
		else
			break;

	/* convert decimal minutes to decimal degrees */
	result /= 60;

	/* retrieve degrees */
	for (i=dot-3, factor=1; i>=begin ;i--, factor *= 10)
		result += (s[i]-'0')*factor;

	/* set to negative if Southing or Westing */
	if (s[0] == 'S' || s[0] == 'W')
		result = -result;

	/* return the decimal degree coordinate */
	return result;
}
