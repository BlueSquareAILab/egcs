#ifndef TONKEY_HPP
#define TONKEY_HPP

#define MAXTOKENS 64


class tonkey
{
private:
    /* data */
    String mTokens[MAXTOKENS];
    int mTokenCount = 0;
    
    
public:
    
    tonkey(/* args */);
    ~tonkey();

    void parse(String _strLine);

    int getTokenCount() { return mTokenCount; }
    String getToken(int _index) { return mTokens[_index]; }

};

tonkey::tonkey(/* args */)
{
}

tonkey::~tonkey()
{
}

void tonkey::parse(String _strLine)
{
    int StringCount = 0;
    // parse command
    while (_strLine.length() > 0)
    {
        int index = _strLine.indexOf(' ');
        if (index == -1) // No space found
        {
            mTokens[StringCount++] = _strLine;
            break;
        }
        else
        {
            mTokens[StringCount++] = _strLine.substring(0, index);
            _strLine = _strLine.substring(index + 1);
        }
    }

    mTokenCount = StringCount;
}

#endif // TONKEY_HPP