/**
 *  Ixion
 */
package fr.ixion.pi;

/**
 * @author Ixion
 */
public enum ComType
{
    MES("[MES]"),
    ERR("[ERR]"),
    COM("[COM]");
    String label;

    ComType(String label)
    {
        this.label = label;
    }

    public String getLabel()
    {
        return label;
    }
}
