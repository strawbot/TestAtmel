#include <QtTest>

// Test each part of the encoder with its counterpart
// group parts as modules and test to counterpart
// replace parts in module with mathematical equivelant
// check open source for decoder elements
// or each test could add to the chainc

class echo : public QObject
{
    Q_OBJECT

    #define MANT_PDU1 24

public:
    char airLinkPdu[MANT_PDU1];

    echo()  { }
    ~echo() { }

    // utility
    void fill_pdu(char * pdu, char length) {
        while (length)  *pdu++ = (length++ * 3);
    }

    // encoder/decoder
    char * encode(char * pdu) {
        return pdu++;
    }

    char * decode(char * pdu) {
        return pdu;
    }

private slots:
    void initTestCase() { fill_pdu(airLinkPdu, MANT_PDU1); }
    void cleanupTestCase() { }
    void echo_verify() {
        QCOMPARE(airLinkPdu, decode(encode(airLinkPdu)));
    }

};

QTEST_APPLESS_MAIN(echo)

#include "tst_echo.moc"
