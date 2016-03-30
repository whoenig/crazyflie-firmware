
// We can have any C++ code here
// Avoid virtual, exceptions, and dynamic allocation for
// highest performance
class A
{
public:
	A(int value)
		: m_value(value)
	{
	}

protected:
	int m_value;
};

class B
	: public A
{
public:
	B()
		: A(0)
	{
	}

	float getValue() {
		return m_value;
	}

};

// We need an extern "C" function to be called from C-code
// The function can use any C++ construct.
extern "C" float testCpp(void)
{
	B b;
	return b.getValue();
}
